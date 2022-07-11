package app

import (
	"fmt"
	"os"
	"sort"

	"github.com/sirupsen/logrus"
)

type Application struct {
	logger    *logrus.Entry
	logfile   *os.File
	config    *Configure
	listToRun map[string]Item
}

type kv struct {
	Key   string
	Value Item
}

func rankMapStringItem(values map[string]Item, asc bool) []kv {
	var ranked []kv
	for k, v := range values {
		ranked = append(ranked, kv{k, v})
	}
	sort.Slice(ranked, func(i, j int) bool {
		if asc {
			return ranked[i].Value.StartNum() < ranked[j].Value.StartNum()
		} else {
			return ranked[i].Value.StartNum() > ranked[j].Value.StartNum()
		}
	})
	return ranked
}

func (a *Application) Start() error {

	err := a.exeStartList(a.listToRun)

	if err != nil {
		a.logger.Error(err)
	} else {
		a.logger.Info("Application is started")
	}

	return err
}

func (a *Application) Stop() error {
	err := a.exeStopList(a.listToRun)

	if err != nil {
		a.logger.Error(err)
	} else {
		a.logger.Info("Application is stopped")
	}

	a.logfile.Close()

	return err
}

func (a *Application) exeStartList(list map[string]Item) error {
	revertList := map[string]Item{}
	ranked := rankMapStringItem(list, true)
	for _, v := range ranked {
		if v.Value.Enabled() {
			if err := v.Value.Start(); err != nil {
				return fmt.Errorf("%v: %w", err, a.exeStopList(revertList))
			}
			a.logger.Debugf("Start [%s] : %d", v.Key, v.Value.StartNum())
		}
		revertList[v.Key] = v.Value
	}

	return nil
}

func (a *Application) exeStopList(list map[string]Item) error {
	var allErr error
	ranked := rankMapStringItem(list, false)

	for _, v := range ranked {
		if v.Value.IsRun() {
			if err := v.Value.Stop(); err != nil {
				allErr = fmt.Errorf("%v: %w", allErr, err)
			}
			a.logger.Debugf("Stop [%s] : %d", v.Key, v.Value.StartNum())
		}
	}

	return allErr
}

type Item interface {
	StartNum() int
	Enabled() bool
	IsRun() bool
	Start() error
	Stop() error
	InitLog(logDir string, file *os.File) error
}

type Configure struct {
	Title               string `toml:"Title"`
	Master              bool   `toml:"Master"`
	Protocol            string `toml:"Protocol"`
	LocalAddr           string `toml:"LocalAddr"`
	LocalPort           int    `toml:"LocalPort"`
	RemoteAddr          string `toml:"RemoteAddr"`
	RemotePort          int    `toml:"RemotePort"`
	Interface           string `toml:"Interface"`
	VirtualIP           string `toml:"VirtualIP"`
	CA                  string `toml:"CAFile"`
	CertFile            string `toml:"CertFile"`
	KeyFile             string `toml:"KeyFile"`
	StorageMountPoint   string `toml:"StorageMountPoint"`
	SmbMountUnit        string `toml:"SmbMountUnit"`
	StorageMountTimeout int    `toml:"SmbMountUnit"`
	LogPath             string `toml:"LogPath"`
	LogFile             string `toml:"LogFile"`
	LogLevel            string `toml:"LogLevel"`
}
