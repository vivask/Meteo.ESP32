package app

import (
	"context"
	"fmt"
	"log"
	"os"
	"time"

	"meteo-daemon/client"
	"meteo-daemon/leader"
	"meteo-daemon/services/auth"
	"meteo-daemon/services/cluster"
	"meteo-daemon/services/data"
	"meteo-daemon/services/mail"
	"meteo-daemon/services/metrics"
	"meteo-daemon/services/mikrotik"
	"meteo-daemon/services/proxy"
	"meteo-daemon/services/scheduler"
	"meteo-daemon/services/telegram"
	"meteo-daemon/services/web"
	"meteo-daemon/startstop"

	"github.com/sirupsen/logrus"
)

var activeServices = map[string]Item{}

func New(cnf *Config, workDir string) (*Application, error) {

	appLogger := logrus.New().WithField("service", cnf.App.Title)

	leader := leader.New(cnf.App.Master)

	cParams := &client.Params{
		Logger:     appLogger,
		Lead:       leader,
		Protocol:   cnf.App.Protocol,
		LocalAddr:  cnf.App.LocalAddr,
		LocalPort:  cnf.App.LocalPort,
		RemoteAddr: cnf.App.RemoteAddr,
		RemotePort: cnf.App.RemotePort,
		CA:         cnf.App.CA,
		CrtClient:  cnf.App.CertFile,
		KeyClient:  cnf.App.KeyFile,
	}

	client, err := client.New(cParams)
	if err != nil {
		return nil, err
	}

	conn, err := data.NewConnection(cnf.Database)
	if err != nil {
		return nil, err
	}

	dParams := &data.Params{
		Hasp:   startstop.New(2 * time.Second),
		Logger: logrus.New().WithField("service", cnf.Database.Title),
		Conn:   conn,
		Cli:    client,
		Lead:   leader,
	}
	repo := data.NewMysqlRepository(cnf.Database, dParams)
	err = repo.Initialize(context.Background())
	if err != nil {
		return nil, fmt.Errorf("Initialize repository fail: %w", err)
	}
	activeServices["db"] = repo

	mtr, err := metrics.New()
	if err != nil {
		return nil, err
	}

	clusterParams := &cluster.Params{
		Hasp:       startstop.New(2 * time.Second),
		Logger:     logrus.New().WithField("service", cnf.Cluster.Title),
		Cli:        client,
		Lead:       leader,
		LocalAddr:  cnf.App.LocalAddr,
		LocalPort:  cnf.App.LocalPort,
		RemoteAddr: cnf.App.RemoteAddr,
		RemotePort: cnf.App.RemotePort,
		CA:         cnf.App.CA,
		CrtClient:  cnf.App.CertFile,
		KeyClient:  cnf.App.KeyFile,
	}
	cluster, err := cluster.New(cnf.Cluster, clusterParams)
	if err != nil {
		return nil, err
	}
	activeServices["cluster"] = cluster

	mailParams := &mail.Params{
		Hasp:   startstop.New(2 * time.Second),
		Logger: logrus.New().WithField("service", cnf.Mail.Title),
	}
	if cnf.Mail.Active {
		activeServices["mail"] = mail.New(cnf.Mail, mailParams)
	}

	mParams := &mikrotik.Params{
		Hasp:   startstop.New(1 * time.Second),
		Logger: logrus.New().WithField("service", cnf.Mikrotik.Title),
		Repo:   repo,
		Lead:   leader,
	}
	servMikrotik := mikrotik.New(cnf.Mikrotik, mParams)
	activeServices["mikrotiks"] = mikrotik.New(cnf.Mikrotik, mParams)

	tParams := &telegram.Params{
		Hasp:   startstop.New(1 * time.Second),
		Logger: logrus.New().WithField("service", cnf.Telegram.Title),
	}
	servTelegram := telegram.New(cnf.Telegram, tParams)
	activeServices["telegram"] = servTelegram

	pParams := proxy.Params{
		Hasp:    startstop.New(2 * time.Second),
		Logger:  logrus.New().WithField("service", cnf.Proxy.Title),
		Repo:    repo,
		Cli:     client,
		WorkDir: workDir,
		Lead:    leader,
	}
	servProxy := proxy.New(cnf.Proxy, &pParams)
	activeServices["proxy"] = servProxy

	blocklists := NewBlocklistsTask(servProxy)
	powercom := NewPowercom(appLogger)
	stor := &StorageMount{
		cli:          client,
		logger:       appLogger,
		mountPoint:   cnf.App.StorageMountPoint,
		mountUnit:    cnf.App.SmbMountUnit,
		timeoutMount: cnf.App.StorageMountTimeout,
	}
	sync := NewSyncEsp32Tables(repo)

	sParams := scheduler.Params{
		Hasp:   startstop.New(1 * time.Second),
		Logger: logrus.New().WithField("service", cnf.Scheduler.Title),
		Repo:   repo,
		Cli:    client,
		Items: map[string]scheduler.SheduleItem{
			"mikrotiks":  servMikrotik,
			"telegram":   servTelegram,
			"blocklists": blocklists,
			"storMount":  stor,
			"powercom":   powercom,
			"syncdb":     sync,
		},
		Lead: leader,
	}
	servScheduler := scheduler.New(cnf.Scheduler, &sParams)
	activeServices["scheduler"] = servScheduler

	auth := auth.New(startstop.New(1*time.Second), logrus.New().WithField("service", cnf.AuthService.Title), cnf.AuthService)

	validator := data.NewValidation()

	ah := auth.NewAuthHandler(validator, repo, auth, activeServices["mail"].(mail.MailService))

	activeServices["auth"] = auth

	wParams := &web.Params{
		Hasp:        startstop.New(2 * time.Second),
		Logger:      logrus.New().WithField("service", cnf.WebServer.Title),
		Protocol:    cnf.App.Protocol,
		Mtr:         mtr,
		WorkDir:     workDir,
		CA:          cnf.App.CA,
		Repo:        repo,
		Lead:        leader,
		Cli:         client,
		Mikrotik:    servMikrotik,
		Telegram:    servTelegram,
		Scheduler:   servScheduler,
		Proxy:       servProxy,
		AuthHandler: ah,
	}

	activeServices["web"] = web.New(cnf.WebServer, wParams)

	app := &Application{
		config:    cnf.App,
		listToRun: activeServices,
	}

	//file, err := os.OpenFile(cnf.App.LogPath+"/"+cnf.App.LogFile, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0666)
	file, err := os.OpenFile(cnf.App.LogPath+"/"+cnf.App.LogFile, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0666)
	if err != nil {
		return nil, err
	}
	appLogger.Logger.SetOutput(file)
	level, err := logrus.ParseLevel(cnf.App.LogLevel)
	if err != nil {
		file.Close()
		return nil, err
	}
	appLogger.Logger.SetLevel(level)

	app.logger = appLogger
	app.logfile = file

	for key, s := range app.listToRun {
		err = s.InitLog(cnf.App.LogPath, file)
		if err != nil {
			appLogger.Errorf("%s:failed logging initialize: %v", key, err)
			return nil, err
		}
	}

	return app, nil
}

func Require(services ...string) {
	for _, service := range services {
		if _, ok := activeServices[service]; !ok {
			log.Fatalf("Требуемый сервис [%s] не активен", service)
		}
	}
}
