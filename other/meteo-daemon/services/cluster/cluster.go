package cluster

import (
	"context"
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/utils"
	"os"
	"sync"

	vip "meteo-daemon/services/cluster/internal"

	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp       domain.StartStopInterface
	Logger     *logrus.Entry
	Cli        *client.Client
	Lead       *leader.Leader
	LocalAddr  string
	LocalPort  int
	RemoteAddr string
	RemotePort int
	CA         string
	CrtClient  string
	KeyClient  string
}

type Cluster struct {
	hasp       domain.StartStopInterface
	logger     *logrus.Entry
	logfile    *os.File
	config     *Config
	cli        *client.Client
	lead       *leader.Leader
	ctx        context.Context
	cancel     context.CancelFunc
	local      string
	remote     string
	vipManager *vip.VIPManager
	stop       chan struct{}
	mux        sync.Mutex
	udpSever   *vip.UdpServer
}

func New(cnf *Config, p *Params) (*Cluster, error) {
	ctx, cancel := context.WithCancel(context.Background())
	return &Cluster{
		hasp:     p.Hasp,
		logger:   p.Logger,
		logfile:  nil,
		config:   cnf,
		cli:      p.Cli,
		lead:     p.Lead,
		local:    fmt.Sprintf("%s:%d", p.LocalAddr, p.LocalPort),
		remote:   fmt.Sprintf("%s:%d", p.RemoteAddr, p.RemotePort),
		ctx:      ctx,
		cancel:   cancel,
		stop:     make(chan struct{}),
		udpSever: vip.NewUdpServer(ctx, p.Logger, cnf.BindAddr, cnf.BindPort),
	}, nil
}

func (c *Cluster) StartNum() int {
	return 2
}

func (c *Cluster) Enabled() bool {
	return true
}

func (c *Cluster) IsRun() bool {
	return c.hasp.IsRun()
}

func (c *Cluster) Start() (err error) {
	if !c.hasp.Start() {
		return fmt.Errorf("%s:failed to start", c.config.Title)
	}

	err = utils.StartListenService(c.udpSever.Listen, 1, "bind cluster port")
	if err != nil {
		return fmt.Errorf("can't bind cluster port: %v", err)
	}

	c.logger.Infof("Listen: %s:%d", c.config.BindAddr, c.config.BindPort)

	err = utils.StartListenService(c.startupVirtualIP, 2, "virtual ip manager")
	if err != nil {
		return fmt.Errorf("%s: failed to start, error: %w", c.config.Title, err)
	}

	c.logger.Debugf("%s: success started", c.config.Title)

	return nil
}

func (c *Cluster) Stop() error {
	if !c.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", c.config.Title)
	}

	c.cancel()
	c.vipManager.Stop()

	c.logger.Debugf("%s: success stoped", c.config.Title)

	if c.logfile != nil {
		c.logfile.Close()
	}

	return nil
}

func (c *Cluster) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, c.config.LogFile, c.config.LogLevel, file, c.logger)
	if err != nil {
		return err
	}
	c.logfile = f
	return nil
}

type Config struct {
	Title     string `toml:"Title"`
	BindAddr  string `toml:"BindAddr"`
	BindPort  int    `toml:"BindPort"`
	Interface string `toml:"Interface"`
	VirtualIP string `toml:"VirtualIP"`
	LogFile   string `toml:"LogFile"`
	LogLevel  string `toml:"LogLevel"`
}
