package pkg

import (
	"meteo-daemon/leader"
	"net/http"
	"time"

	"github.com/sirupsen/logrus"
)

type VIPHandler func(alive, leader bool)

type Params struct {
	Logger  *logrus.Entry
	Lead    *leader.Leader
	Remote  string
	Handler VIPHandler
}

type VIPManager struct {
	logger              *logrus.Entry
	stop                chan bool
	finished            chan bool
	networkConfigurator NetworkConfigurator
	request             *http.Request
	lead                *leader.Leader
	port                string
	remoteAlive         bool
	handler             VIPHandler
	udpClient           UdpClient
}

func NewVIPManager(networkConfigurator NetworkConfigurator, p *Params) *VIPManager {
	return &VIPManager{
		logger:              p.Logger,
		networkConfigurator: networkConfigurator,
		lead:                p.Lead,
		port:                p.Remote,
		handler:             p.Handler,
		udpClient:           *NewUdpClient(p.Logger, p.Remote),
	}
}

func (manager *VIPManager) addIP(verbose bool) {
	if error := manager.networkConfigurator.AddIP(); error != nil {
		manager.logger.WithFields(logrus.Fields{"error": error, "ip": manager.networkConfigurator.IP(), "interface": manager.networkConfigurator.Interface()}).Error("Could not set ip")
	} else if verbose {
		manager.logger.WithFields(logrus.Fields{"ip": manager.networkConfigurator.IP(), "interface": manager.networkConfigurator.Interface()}).Info("Added IP")
	}
}

func (manager *VIPManager) deleteIP(verbose bool) {
	if error := manager.networkConfigurator.DeleteIP(); error != nil {
		manager.logger.WithFields(logrus.Fields{"error": error, "ip": manager.networkConfigurator.IP(), "interface": manager.networkConfigurator.Interface()}).Error("Could not delete ip")
	} else if verbose {
		manager.logger.WithFields(logrus.Fields{"ip": manager.networkConfigurator.IP(), "interface": manager.networkConfigurator.Interface()}).Info("Deleted IP")
	}
}

func (manager *VIPManager) Start() error {
	manager.stop = make(chan bool, 1)
	manager.finished = make(chan bool, 1)
	ticker := time.NewTicker(time.Second)

	manager.lead.SetLeader(false)

	manager.deleteIP(false)

	go func() {
		for {
			select {
			case <-ticker.C:
				const pack = "test"
				echo, err := manager.udpClient.Dial(pack)
				alive := (err == nil && echo == pack)
				if manager.remoteAlive != alive {
					if alive {
						manager.logger.Info("Remote server is alive")
					} else {
						manager.logger.Info("Remote server is dead")
					}
					manager.remoteAlive = alive
					manager.lead.SetAliveRemote(alive)
				}
				if !alive {
					if !manager.lead.IsLeader() {
						manager.lead.SetLeader(true)
						manager.logger.Info("LEADING")
						manager.addIP(true)
						manager.handler(manager.remoteAlive, true)
					}
				} else {
					if manager.lead.IsMaster() {
						if !manager.lead.IsLeader() {
							manager.lead.SetLeader(true)
							manager.logger.Info("LEADING")
							manager.addIP(true)
							manager.handler(manager.remoteAlive, true)
						}
					} else {
						if manager.lead.IsLeader() {
							manager.lead.SetLeader(false)
							manager.logger.Info("FOLLOWING")
							manager.deleteIP(true)
							manager.handler(manager.remoteAlive, false)
						}
					}
				}
			case <-manager.stop:
				manager.logger.Debug("Virtual IP Manager Stopping")

				if manager.lead.IsLeader() {
					manager.deleteIP(true)
				}

				close(manager.finished)

				return
			}
		}
	}()

	manager.logger.Debug("Virtual IP Manager Started")

	return nil
}

func (manager *VIPManager) Stop() {
	close(manager.stop)

	<-manager.finished

	manager.logger.Info("Stopped")
}
