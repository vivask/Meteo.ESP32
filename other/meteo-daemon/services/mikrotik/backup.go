package mikrotik

import (
	"errors"
	"fmt"
	"meteo-daemon/utils"
	"net"
	"os"
	"path/filepath"
	s "strings"
	"time"

	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

func (m *Mikrotik) Backup() (err error) {

	var p sshParams
	var keyErr *knownhosts.KeyError
	timeout := time.After(SSH_TIMEOUT * time.Second)
	kh, err := m.checkKnownHosts()
	if err != nil {
		return fmt.Errorf("check knownhosts error: %w", err)
	}

	config := &ssh.ClientConfig{
		//User: conf.Mikrotik.User,
		Auth: m.makeKeyring(),
		HostKeyCallback: ssh.HostKeyCallback(func(host string, remote net.Addr, pubKey ssh.PublicKey) error {
			hErr := kh(host, remote, pubKey)
			if errors.As(hErr, &keyErr) && len(keyErr.Want) > 0 {
				m.logger.Infof("WARNING: %v is not a key of %s, either a MiTM attack or %s has reconfigured the host pub key.", hostKeyString(pubKey), host, host)
				return keyErr
			} else if errors.As(hErr, &keyErr) && len(keyErr.Want) == 0 {
				m.logger.Infof("WARNING: %s is not trusted, adding this key: %q to known_hosts file.", host, hostKeyString(pubKey))
				return m.addHostKey(host, remote, pubKey)
			}
			return nil
		}),
	}

	p.init(config, "export verbose")
	m.executeAllHosts(&p)

	for i := 0; i < p.hosts; i++ {
		select {
		case r0 := <-p.req:
			verbose := r0.request
			host := r0.host
			if len(verbose) == 0 {
				m.logger.Errorf("Received a zero length response from the host: %s\n", host)
			} else {
				idx := s.Index(verbose, "\n")
				verbose = utils.LeftTrunc(verbose, idx+1)
				configFile := m.config.GitRepo + "/" + host
				_, err := os.Stat(configFile)
				if err == nil {
					savedMD5, err := utils.GetMD5FileSum(configFile)
					if err != nil {
						return fmt.Errorf("GetMD5FileSum error: %w", err)
					}
					receivedMD5 := utils.GetMD5StringlnSum(verbose)
					if savedMD5 != receivedMD5 {
						m.logger.Infof("Backuped: %s", configFile)
						m.saveConfig(configFile, verbose)
						err = m.downloadBackupFile(r0.host, r0.port, config)
						if err != nil {
							m.logger.Error(err)
						}
					}
				} else {
					m.logger.Infof("Backuped: %s", configFile)
					m.saveConfig(configFile, verbose)
					err = m.downloadBackupFile(r0.host, r0.port, config)
					if err != nil {
						m.logger.Error(err)
					}
				}
			}
		case <-timeout:
			m.logger.Error("Timed out ssh request")
		}
	}
	p.close()

	err = m.gitPush(m.config.GitRepo)
	if err != nil {
		return fmt.Errorf("gitPush error: %w", err)
	}
	files, err := filepath.Glob(m.config.GitRepo + "/*.backup")
	if err != nil {
		return fmt.Errorf("Glob error: %w", err)
	}
	for _, f := range files {
		if err := os.Remove(f); err != nil {
			return fmt.Errorf("remove file [%s] error: %w", f, err)
		}
	}
	return nil
}

func (m *Mikrotik) downloadBackupFile(host, port string, config *ssh.ClientConfig) (err error) {
	rFileName := host + "-" + time.Now().Format(time.RFC3339)
	cmd := "/system backup save name=" + rFileName
	okReq := "Configuration backup saved"
	_, err = m.execMikrotik(host, port, config.User, cmd, okReq, config)
	if err != nil {
		return
	}
	rBackupFile := rFileName + ".backup"
	sftpReq := make(chan error, 1)
	go func(sftpReq chan error) {
		sftpReq <- m.sftpDownload(rBackupFile, m.config.GitRepo+"/"+rBackupFile, host, port, config)
	}(sftpReq)
	timeout := time.After(SSH_TIMEOUT * time.Second)
	select {
	case sftpErr := <-sftpReq:
		if sftpErr != nil {
			return fmt.Errorf("Unable to download remote file: %s\n", rBackupFile)
		}
	case <-timeout:
		return fmt.Errorf("Timed out file download from: %s", host)
	}
	cmd = "/file remove \"" + rBackupFile + "\""
	okReq = host + ": $"
	_, err = m.execMikrotik(host, port, config.User, cmd, okReq, config)
	if err != nil {
		return
	}
	m.logger.Debugf("backup file download success")
	return
}
