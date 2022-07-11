package proxy

import (
	"context"
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/services/data"
	u "meteo-daemon/utils"
	"os"
	"time"

	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp    domain.StartStopInterface
	Logger  *logrus.Entry
	Repo    data.Repository
	Cli     *client.Client
	WorkDir string
	Lead    *leader.Leader
}

type Proxy struct {
	hasp    domain.StartStopInterface
	logger  *logrus.Entry
	config  *Config
	workdir string
	logfile *os.File
	repo    data.Repository
	cli     *client.Client
	server  *Server
	ctx     context.Context
	cancel  context.CancelFunc
	lead    *leader.Leader
}

func New(cnf *Config, p *Params) *Proxy {
	return &Proxy{
		hasp:    p.Hasp,
		logger:  p.Logger,
		config:  cnf,
		workdir: p.WorkDir,
		repo:    p.Repo,
		cli:     p.Cli,
		server:  NewServer(p.Logger, cnf),
		lead:    p.Lead,
	}
}

func (p *Proxy) checkLoadedBlackLists(loaded []bool) {
	for idx, load := range loaded {
		if !load {
			message := "Не удалось загрузить список блокировки: " + p.config.Blocklist[idx]
			_, err := p.cli.PostInt("/verify/telegram/rest/message", message)
			if err != nil {
				p.logger.Errorf("can't send telegram message: %v", err)
			}
		}
	}
}

func (p *Proxy) TaskBlacklistLoad() {
	if p.lead.IsLeader() {
		p.logger.Debug("Download blocklists starting...")
		list, loaded := UpdateFile(p.logger, p.config.Blocklist, p.workdir)
		p.server.SetBlackList(list)
		p.checkLoadedBlackLists(loaded)
		if p.lead.IsLeader() {
			_, err := p.cli.PostExt("/verify/proxy/rest/blklist/update", list.data)
			if err != nil {
				p.logger.Errorf("update external blocklist error: %v", err)
			}
		}
		p.logger.Debug("Download blocklists complet")
	}
}

func (p *Proxy) ReplaceBlocklist(hosts map[string]struct{}) {
	p.server.bl.data = hosts
	err := p.server.bl.SaveToFile(p.workdir)
	if err != nil {
		p.logger.Error("replace blocklist error: %v", err)
		return
	}
	p.logger.Infof("Loaded %d block hosts from external", p.server.bl.Count())
}

func (p *Proxy) verifyServerState() {
	for {
		timer := time.After(timeVerifyRetry)
		select {
		case <-timer:
			p.server.verifyState()
		case <-p.ctx.Done():
			return
		}
	}
}

func (Proxy) StartNum() int {
	return 8
}

func (p *Proxy) Enabled() bool {
	return p.config.Active
}

func (p *Proxy) IsRun() bool {
	return p.hasp.IsRun()
}

func (p *Proxy) Start() (err error) {
	if !p.hasp.Start() {
		return fmt.Errorf("%s:failed to start", p.config.Title)
	}

	err = p.configLocalResolver()
	if err != nil {
		return fmt.Errorf("can't configure resolver: %w", err)
	}

	p.server.SetZones(LoadZones(p.logger, p.repo))
	p.server.SetBlackList(LoadFromFile(p.logger, p.workdir))
	p.server.SetUnlocker(LoadUnlocker(p.logger, p.repo, p.cli))

	p.ctx, p.cancel = context.WithCancel(context.Background())
	timeout := time.After(1 * time.Second)
	fErr := make(chan error, 1)
	go func(fErr chan error) {
		fErr <- p.server.Run(p.ctx)
	}(fErr)

	select {
	case e := <-fErr:
		if e != nil {
			return fmt.Errorf("%s:failed to start", p.config.Title)
		}
	case <-timeout:
	}

	go p.verifyServerState()

	/*if p.lead.IsMaster() {
		go func() {
			time.Sleep(5 * time.Second)
			p.repo.ReplaceAllExt()
		}()
	}*/

	p.logger.Debugf("%s: success started", p.config.Title)

	return nil
}

func (p *Proxy) Stop() error {
	if !p.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", p.config.Title)
	}

	p.cancel()
	p.logger.Debugf("%s: success stoped", p.config.Title)

	return nil
}

func (p *Proxy) InitLog(logDir string, file *os.File) error {
	f, err := u.InitLogrus(logDir, p.config.LogFile, p.config.LogLevel, file, p.logger)
	if err != nil {
		return err
	}
	p.logfile = f
	return nil
}

func (p *Proxy) ChangeBlackList() {
	if p.server.blkListOn {
		p.server.blkListOn = false
		p.logger.Info("Block List Deactivated")
	} else {
		p.server.blkListOn = true
		p.logger.Info("Block List Activated")
	}
}

func (p *Proxy) ChangeCache() {
	if p.server.cacheOn {
		p.server.CacheClear()
		p.server.cacheOn = false
		p.logger.Info("Cache Deactivated")
	} else {
		p.server.cacheOn = true
		p.logger.Info("Cache Activated")
	}
}

func (p *Proxy) ChangeUnlocker() {
	if p.server.unlockerOn {
		p.server.unlockerOn = false
		p.logger.Info("Unlocker Deactivated")
	} else {
		p.server.unlockerOn = true
		p.logger.Info("Unlocker Activated")
	}
}

func (p *Proxy) DeleteAutoHost(id string) error {
	return p.server.un.DeleteHost(id)
}

func (p *Proxy) IgnoreAutoHost(id string) error {
	return p.server.un.IgnoreHost(id)
}

func (p *Proxy) DeleteIgnore(id string) error {
	return p.server.un.DeleteIgnore(id)
}

func (p *Proxy) RestoreIgnored(id string) error {
	return p.server.un.RestoreHost(id)
}

func (p *Proxy) InsertManual(host data.ToVpnManual) error {
	return p.server.un.InsertManual(host)
}

func (p *Proxy) UpdateManual(host data.ToVpnManual) error {
	return p.server.un.UpdateManual(host)
}

func (p *Proxy) RemoveManual(id uint32) error {
	return p.server.un.RemoveManual(id)
}

func (p *Proxy) GetState() *data.ProxyState {
	return &data.ProxyState{
		Active:     true,
		BlkListOn:  p.server.blkListOn,
		CacheOn:    p.server.cacheOn,
		UnlockerOn: p.server.unlockerOn,
	}
}

func (p *Proxy) SyncZones(host data.Homezone) {
	if len(host.IPv4) == 0 {
		p.server.zo.RemoveHost(host.DomainName)
	} else {
		p.server.zo.UpdateHost(host.DomainName, host.IPv4)
	}
}

func (p *Proxy) ReloadZones() {
	p.server.SetZones(LoadZones(p.logger, p.repo))
}

func (p *Proxy) AddHomeZoneHost(name, ip string) {
	p.server.zo.AddHost(name, ip)
}

func (p *Proxy) UpdateHomeZoneHost(name, ip string) {
	p.server.zo.UpdateHost(name, ip)
}

func (p *Proxy) RemoveHomeZoneHost(name string) {
	p.server.zo.RemoveHost(name)
}

type Config struct {
	Title          string   `toml:"Title"`
	Active         bool     `toml:"Active"`
	Listen         string   `toml:"Listen"`
	UDPPort        int      `toml:"UDPPort"`
	TCPPort        int      `toml:"TCPPort"`
	EvictMetrics   bool     `toml:"EvictMetrics"`
	VpnNS          []string `toml:"VpnNS"`
	DirectNS       []string `toml:"DirectNS"`
	ProviderNS     []string `toml:"ProviderNS"`
	LocalResolvers []string `toml:"LocalResolvers"`
	Cached         bool     `toml:"Cached"`
	CacheSize      int      `toml:"CacheSize"`
	Unlocker       bool     `toml:"Unlocker"`
	HostsFile      string   `toml:"HostsFile"`
	BlockListOn    bool     `toml:"BlockListOn"`
	Blocklist      []string `toml:"Blocklist"`
	UpdateInterval string   `toml:"UpdateInterval"`
	UpdateTime     string   `toml:"UpdateTime"`
	BlockAddress4  string   `toml:"BlockAddress4"`
	BlockAddress6  string   `toml:"BlockAddress6"`
	LogFile        string   `toml:"LogFile"`
	LogLevel       string   `toml:"LogLevel"`
}
