package cluster

import (
	"fmt"
	"strings"
	"time"

	vip "meteo-daemon/services/cluster/internal"

	"github.com/sirupsen/logrus"
)

var reloadSchedulerRunning = false
var masterStart = false

func (c *Cluster) startupVirtualIP() error {

	c.logger.WithFields(logrus.Fields{"virtual-ip": c.config.VirtualIP, "interface": c.config.Interface}).Info("Network")

	netlinkNetworkConfigurator, err := vip.NewNetlinkNetworkConfigurator(c.config.VirtualIP, c.config.Interface)
	if err != nil {
		return fmt.Errorf("network failure: %w", err)
	}
	split := strings.Split(c.remote, ":")
	paramas := &vip.Params{
		Logger:  c.logger,
		Lead:    c.lead,
		Remote:  fmt.Sprintf("%s:%d", split[0], c.config.BindPort),
		Handler: c.Handler,
	}

	c.vipManager = vip.NewVIPManager(netlinkNetworkConfigurator, paramas)

	if err := c.vipManager.Start(); err != nil {
		return fmt.Errorf("start failed: %w", err)
	}

	select {
	case <-c.ctx.Done():
		return nil
	}
}

func (c *Cluster) Handler(alive, leader bool) {
	if !c.lead.IsMaster() {
		go c.reloadScheduler()
	}
}

func (c *Cluster) setStateReloadScheduler(state bool) {
	c.mux.Lock()
	reloadSchedulerRunning = state
	c.mux.Unlock()
}

func (c *Cluster) getStateReloadScheduler() bool {
	c.mux.Lock()
	defer c.mux.Unlock()
	return reloadSchedulerRunning
}

func (c *Cluster) reloadScheduler() {
	if c.getStateReloadScheduler() {
		c.setStateReloadScheduler(false)
		c.stop <- struct{}{}
		return
	}
	c.setStateReloadScheduler(true)

	timer := time.After(5 * time.Second)
	select {
	case <-timer:
		_, err := c.cli.PutInt("/verify/schedule/rest/job/reload", nil)
		if err != nil {
			c.logger.Error(err)
		} else {
			c.logger.Info("Sheduler reloaded")
		}
	case <-c.stop:
	}
	c.setStateReloadScheduler(false)
}
