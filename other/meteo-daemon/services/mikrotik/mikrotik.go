package mikrotik

import (
	"bytes"
	"context"
	"encoding/base64"
	"fmt"
	"io"
	"io/ioutil"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/services/data"
	"meteo-daemon/utils"
	"net"
	"os"
	"regexp"
	"time"

	"github.com/pkg/sftp"
	"github.com/sirupsen/logrus"
	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

type Params struct {
	Hasp   domain.StartStopInterface
	Logger *logrus.Entry
	Repo   data.Repository
	Lead   *leader.Leader
}

const SSH_TIMEOUT = 20

type sshRequest struct {
	request string
	host    string
	port    string
}

type sshParams struct {
	conf  *ssh.ClientConfig
	req   chan sshRequest
	cmd   string
	hosts int
}

func (p *sshParams) init(cfg *ssh.ClientConfig, cmd string) {
	p.conf = cfg
	p.cmd = cmd
	p.req = make(chan sshRequest, 10)
}

func (p *sshParams) close() {
	close(p.req)
}

type Mikrotik struct {
	hasp    domain.StartStopInterface
	logger  *logrus.Entry
	logfile *os.File
	config  *Config
	repo    data.Repository
	lead    *leader.Leader
}

func New(cnf *Config, p *Params) *Mikrotik {
	return &Mikrotik{
		hasp:   p.Hasp,
		logger: p.Logger,
		config: cnf,
		repo:   p.Repo,
		lead:   p.Lead,
	}
}

func (m *Mikrotik) Run(params map[string]string) (err error) {
	if len(params) != 1 {
		return fmt.Errorf("Invalid number of parameters: %d, required 1", len(params))
	}

	if value, found := params["task"]; found {
		switch value {
		case "backup":
			err = m.Backup()
		case "byfly":
			err = m.CheckPPPState(m.config.PPPHost, m.config.PPPPort, m.config.PPPUser)
		default:
			return fmt.Errorf("Unsupported task: %s", value)
		}
	} else {
		return fmt.Errorf("Required parameter task not found: %v", params)
	}
	return
}

func (m *Mikrotik) Terminate() error {
	return nil
}

func (m *Mikrotik) getSshRequest(hostname string, port string, p *sshParams) {
	var r sshRequest
	r.host = hostname
	r.port = port
	r.request = m.executeCmd(p.cmd, hostname, port, p.conf)
	p.req <- r
}

func (m *Mikrotik) executeAllHosts(p *sshParams) {
	hosts := m.config.Hosts
	ports := m.config.Ports
	users := m.config.Users
	p.hosts = len(hosts)
	i := 0
	for _, hostname := range hosts {
		port := ports[i]
		p.conf.User = users[i]
		go m.getSshRequest(hostname, port, p)
		i++
	}
}

func (m *Mikrotik) saveConfig(fName string, body string) {
	f, err := os.OpenFile(fName, os.O_WRONLY|os.O_CREATE, 0644)
	if err != nil {
		m.logger.Error(err)
		return
	}
	_, err = f.WriteString(body)
	if err != nil {
		m.logger.Error(err)
	}
	f.Close()
}

func makeSigner(finger string) (signer ssh.Signer, err error) {
	signer, err = ssh.ParsePrivateKey([]byte(finger))
	return signer, err
}

/*func makeSigner2(keyname string) (signer ssh.Signer, err error) {
	fp, err := os.Open(keyname)
	if err != nil {
		return
	}
	defer fp.Close()

	buf, err := ioutil.ReadAll(fp)
	signer, err = ssh.ParsePrivateKey(buf)
	return signer, err
}*/

func (m *Mikrotik) makeKeyring() (signers []ssh.AuthMethod) {
	if rows, err := m.repo.GetAllSshKeys(context.Background()); err != nil {
		m.logger.Error("Get keys from repo fail: %v", err)
	} else {
		for _, row := range rows {
			signer, err := makeSigner(row.Finger)
			if err != nil {
				m.logger.Errorf("Key make error from signature: %s, %v\n", row.Owner, err)
			} else {
				signers = append(signers, ssh.PublicKeys(signer))
			}
		}
	}
	return
}

func (m *Mikrotik) hostUsed(hostname string) error {
	err := m.repo.UpTimeKnowhosts(context.Background(), hostname)
	if err != nil {
		return fmt.Errorf("Update timestamp knownhosts fail: %w", err)
	}
	return nil
}

func (m *Mikrotik) executeCmd(cmd, hostname, port string, config *ssh.ClientConfig) string {
	conn, err := ssh.Dial("tcp", hostname+":"+port, config)
	if err != nil {
		m.logger.Error(err)
		return ""
	}
	defer conn.Close()

	session, err := conn.NewSession()
	if err != nil {
		m.logger.Error(err)
		return ""
	}
	defer session.Close()

	err = m.hostUsed(hostname)
	if err != nil {
		m.logger.Error(err)
	}

	var stdoutBuf bytes.Buffer
	session.Stdout = &stdoutBuf
	session.Run(cmd)

	return hostname + ": " + stdoutBuf.String()
}

func (m *Mikrotik) sftpDownload(remoteFile, localFile, hostname, port string, config *ssh.ClientConfig) (err error) {
	conn, err := ssh.Dial("tcp", hostname+":"+port, config)
	if err != nil {
		m.logger.Error(err)
		return
	}
	defer conn.Close()

	sc, err := sftp.NewClient(conn)
	if err != nil {
		m.logger.Errorf("Unable to start SFTP subsystem: %v\n", err)
		return
	}
	defer sc.Close()

	srcFile, err := sc.OpenFile(remoteFile, (os.O_RDONLY))
	if err != nil {
		m.logger.Errorf("Unable to open remote file: %v\n", err)
		return
	}
	defer srcFile.Close()

	dstFile, err := os.Create(localFile)
	if err != nil {
		m.logger.Errorf("Unable to open local file: %v\n", err)
		return
	}
	defer dstFile.Close()

	_, err = io.Copy(dstFile, srcFile)
	if err != nil {
		m.logger.Errorf("Unable to download remote file: %v\n", err)
	}

	return
}

func (m *Mikrotik) sftpUpload(remoteFile, localFile, hostname, port string, config *ssh.ClientConfig) (err error) {
	conn, err := ssh.Dial("tcp", hostname+":"+port, config)
	if err != nil {
		m.logger.Error(err)
		return
	}
	defer conn.Close()

	sc, err := sftp.NewClient(conn)
	if err != nil {
		m.logger.Errorf("Unable to start SFTP subsystem: %v\n", err)
		return
	}
	defer sc.Close()

	srcFile, err := os.Open(localFile)
	if err != nil {
		return
	}
	defer srcFile.Close()

	dstFile, err := sc.Create(remoteFile)
	if err != nil {
		return
	}
	defer dstFile.Close()
	_, err = io.Copy(dstFile, srcFile)

	return
}

func (m *Mikrotik) createKnownHosts() (fName string, err error) {

	rows, err := m.repo.GetAllKnowhosts(context.Background())
	if err != nil {
		return fName, fmt.Errorf("get knownhosts error:% w", err)
	}
	f, err := ioutil.TempFile("/tmp", "knownhosts")
	if err != nil {
		return fName, fmt.Errorf("can't open file %s error: %w", f.Name(), err)
	}
	defer f.Close()
	for _, row := range rows {
		_, err = f.WriteString(fmt.Sprintf("%s\n", row.Finger))
		if err != nil {
			return fName, fmt.Errorf("can't write file %s error: %w", f.Name(), err)
		}
	}
	return f.Name(), nil
}

func (m *Mikrotik) checkKnownHosts() (ssh.HostKeyCallback, error) {
	fName, err := m.createKnownHosts()
	if err != nil {
		return nil, fmt.Errorf("create knownhost file error: %w", err)
	}
	return knownhosts.New(fName)
}

func (m *Mikrotik) addHostKey(host string, remote net.Addr, pubKey ssh.PublicKey) error {

	knownHost := knownhosts.Normalize(remote.String())
	finger := knownhosts.Line([]string{knownHost}, pubKey)

	err := m.repo.AddKnowhost(context.Background(), data.Knowhosts{Host: knownHost, Finger: finger})
	if err != nil {
		return fmt.Errorf("Add knownhost fail: %w", err)
	}
	return nil
}

func (m *Mikrotik) execMikrotik(host, port, username, cmd, okReq string, config *ssh.ClientConfig) (req string, err error) {

	timeout := time.After(SSH_TIMEOUT * time.Second)
	config.User = username
	p := &sshParams{
		conf: config,
		cmd:  cmd,
		req:  make(chan sshRequest, 10),
	}
	defer p.close()

	go m.getSshRequest(host, port, p)

	select {
	case r0 := <-p.req:
		req = r0.request
		matched, _ := regexp.MatchString(okReq, req)
		if !matched {
			return "", fmt.Errorf("Error cmd execution on: %s", req)
		}
	case <-timeout:
		return "", fmt.Errorf("Timed out ssh request")
	}
	return req, nil
}

func (m *Mikrotik) execMikrotikNoCheck(host, port, username, cmd string, config *ssh.ClientConfig) (req string, err error) {

	timeout := time.After(SSH_TIMEOUT * time.Second)
	config.User = username
	p := &sshParams{
		conf: config,
		cmd:  cmd,
		req:  make(chan sshRequest, 10),
	}
	defer p.close()

	go m.getSshRequest(host, port, p)

	select {
	case r0 := <-p.req:
		req = r0.request
	case <-timeout:
		return "", fmt.Errorf("Timed out ssh request")
	}
	return req, nil
}

func hostKeyString(k ssh.PublicKey) string {
	return k.Type() + " " + base64.StdEncoding.EncodeToString(k.Marshal())
}

func (Mikrotik) StartNum() int {
	return 5
}

func (m *Mikrotik) Enabled() bool {
	return m.config.Active
}

func (m *Mikrotik) IsRun() bool {
	return m.hasp.IsRun()
}

func (m *Mikrotik) Start() error {
	if !m.hasp.Start() {
		return fmt.Errorf("%s:failed to start", m.config.Title)
	}

	m.logger.Debugf("%s: success started", m.config.Title)

	return nil
}

func (m *Mikrotik) Stop() error {
	if !m.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", m.config.Title)
	}

	m.logger.Debugf("%s: success stoped", m.config.Title)

	if m.logfile != nil {
		m.logfile.Close()
	}

	return nil
}

func (m *Mikrotik) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, m.config.LogFile, m.config.LogLevel, file, m.logger)
	if err != nil {
		return err
	}
	m.logfile = f
	return nil
}

func (m *Mikrotik) SyncZones(hosts map[string]string) error {
	return m.SyncDNS(m.config.PPPHost, m.config.PPPPort, m.config.PPPUser, hosts)
}

func (m *Mikrotik) PutHostToVpn(host data.ToVpnManual) error {
	return m.PutInVpn(m.config.VPNHost, m.config.VPNPort, m.config.VPNUser, host)
}

func (m *Mikrotik) RemoveHostFromVpn(host data.ToVpnManual) error {
	return m.RemoveFromVpn(m.config.VPNHost, m.config.VPNPort, m.config.VPNUser, host)
}

type Config struct {
	Title     string   `toml:"Title"`
	Active    bool     `toml:"Active"`
	Hosts     []string `toml:"Hosts"`
	Ports     []string `toml:"Ports"`
	Users     []string `toml:"Users"`
	GitRepo   string   `toml:"GitRepo"`
	LogFile   string   `toml:"LogFile"`
	LogLevel  string   `toml:"LogLevel"`
	PPPHost   string   `toml:"PPPHost"`
	PPPPort   string   `toml:"PPPPort"`
	PPPUser   string   `toml:"PPPUser"`
	PPPIface  string   `toml:"PPPIface"`
	PPPScript string   `toml:"PPPScript"`
	VPNHost   string   `toml:"VPNHost"`
	VPNPort   string   `toml:"VPNPort"`
	VPNUser   string   `toml:"VPNUser"`
	VPNList   string   `toml:"VPNList"`
}
