package web

import (
	"crypto/tls"
	"crypto/x509"
	"fmt"
	"html/template"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"strconv"

	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/services/auth"
	"meteo-daemon/services/data"
	"meteo-daemon/services/metrics"
	"meteo-daemon/services/mikrotik"
	"meteo-daemon/services/proxy"
	"meteo-daemon/services/scheduler"
	"meteo-daemon/services/telegram"
	"meteo-daemon/utils"
	u "meteo-daemon/utils"

	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp        domain.StartStopInterface
	Logger      *logrus.Entry
	Protocol    string
	Mtr         *metrics.Metrics
	WorkDir     string
	CA          string
	Repo        *data.MysqlRepository
	Cli         *client.Client
	Lead        *leader.Leader
	Mikrotik    *mikrotik.Mikrotik
	Telegram    *telegram.Telegram
	Scheduler   *scheduler.Scheduler
	Proxy       *proxy.Proxy
	AuthHandler *auth.AuthHandler
}

type Web struct {
	hasp          domain.StartStopInterface
	logger        *logrus.Entry
	protocol      string
	logfile       *os.File
	workdir       string
	ca            string
	config        *Config
	repo          data.Repository
	cli           *client.Client
	lead          *leader.Leader
	auth          *auth.AuthHandler
	metrics       *metrics.Metrics
	templateCache map[string]*template.Template
	mk            *mikrotik.Mikrotik
	tg            *telegram.Telegram
	schd          *scheduler.Scheduler
	px            *proxy.Proxy
}

func New(cnf *Config, p *Params) *Web {

	web := &Web{
		hasp:     p.Hasp,
		logger:   p.Logger,
		protocol: p.Protocol,
		logfile:  nil,
		workdir:  p.WorkDir,
		ca:       p.CA,
		config:   cnf,
		repo:     p.Repo,
		cli:      p.Cli,
		lead:     p.Lead,
		auth:     p.AuthHandler,
		metrics:  p.Mtr,
		mk:       p.Mikrotik,
		tg:       p.Telegram,
		schd:     p.Scheduler,
		px:       p.Proxy,
	}

	return web
}

func (w *Web) runWeb() error {

	listener := w.config.HttpAddr + ":" + strconv.FormatInt(int64(w.config.HttpPort), 10)

	var err error

	templateCache, err := w.newTemplateCache(w.config.UiPath)
	if err != nil {
		w.logger.Fatal(err)
	}

	w.templateCache = templateCache

	switch w.protocol {
	case "http":
		err = http.ListenAndServe(listener, w.routes())
		if err != nil {
			return fmt.Errorf("Failed to start server: %v\n", err)
		}
	case "https":
		tlsMinVersion := tls.VersionTLS12
		switch w.config.TLSMinVersion {
		case "TLS13":
			tlsMinVersion = tls.VersionTLS13
		case "TLS12":
			tlsMinVersion = tls.VersionTLS12
		case "TLS11":
			tlsMinVersion = tls.VersionTLS11
		case "TLS10":
			tlsMinVersion = tls.VersionTLS10
		}

		caCert, err := ioutil.ReadFile(w.ca)
		if err != nil {
			return fmt.Errorf("error read CA: %w", err)
		}
		caCertPool := x509.NewCertPool()
		caCertPool.AppendCertsFromPEM(caCert)

		/*cert, err := tls.LoadX509KeyPair(w.config.CertFile, w.config.KeyFile)
		if err != nil {
			log.Fatalf("server: loadkeys: %s", err)
		}*/
		cfg := &tls.Config{
			ClientAuth: tls.NoClientCert,
			ClientCAs:  caCertPool,
			//Certificates:     []tls.Certificate{cert},
			MinVersion:       uint16(tlsMinVersion),
			CurvePreferences: []tls.CurveID{tls.CurveP521, tls.CurveP384, tls.CurveP256},
		}

		writer := w.logger.Writer()
		defer writer.Close()

		srv := &http.Server{
			Addr:      listener,
			ErrorLog:  log.New(writer, "", 0),
			Handler:   w.routes(),
			TLSConfig: cfg,
		}
		//fCert := w.workdir + "/" + w.config.CertFile
		//fKey := w.workdir + "/" + w.config.KeyFile
		err = srv.ListenAndServeTLS(w.config.CertFile, w.config.KeyFile)
		if err != nil {
			return fmt.Errorf("Failed to start web server: %v\n", err)
		}

	default:
		return fmt.Errorf("Unexpected server protocol: %s", w.protocol)
	}

	return nil
}

func (Web) StartNum() int {
	return 7
}

func (w *Web) Enabled() bool {
	return w.config.Active
}

func (w *Web) IsRun() bool {
	return w.hasp.IsRun()
}

func (w *Web) Start() (err error) {
	if !w.hasp.Start() {
		return fmt.Errorf("%s:failed to start", w.config.Title)
	}

	err = u.StartListenService(w.runWeb, 1, w.config.Title)
	if err != nil {
		return
	}

	w.logger.Debugf("%s: success started", w.config.Title)

	return nil
}

func (w *Web) Stop() error {
	if !w.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", w.config.Title)
	}

	w.logger.Debugf("%s: success stoped", w.config.Title)

	if w.logfile != nil {
		w.logfile.Close()
	}

	return nil
}

func (w *Web) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, w.config.LogFile, w.config.LogLevel, file, w.logger)
	if err != nil {
		return err
	}
	w.logfile = f
	return nil
}

type Config struct {
	Title         string `toml:"Title"`
	Active        bool   `toml:"Active"`
	HttpAddr      string `toml:"HttpAddr"`
	HttpPort      int    `toml:"HttpPort"`
	CertFile      string `toml:"CertFile"`
	KeyFile       string `toml:"KeyFile"`
	TLSMinVersion string `toml:"TLSMinVersion"`
	UiPath        string `toml:"UiPath"`
	ControlMAC    string `toml:"ControlMAC"`
	MAC           string `toml:"MAC"`
	UploadPath    string `toml:"UploadPath"`
	LogFile       string `toml:"LogFile"`
	LogLevel      string `toml:"LogLevel"`
}
