package app

import (
	"fmt"
	"meteo-daemon/services/proxy"
)

type ProxyBlocklistsLoad struct {
	proxy *proxy.Proxy
}

func NewBlocklistsTask(p *proxy.Proxy) *ProxyBlocklistsLoad {
	return &ProxyBlocklistsLoad{
		proxy: p,
	}
}

func (p *ProxyBlocklistsLoad) Run(params map[string]string) (err error) {
	if len(params) != 0 {
		return fmt.Errorf("Invalid number of parameters: %d, required 0", len(params))
	}

	p.proxy.TaskBlacklistLoad()
	return nil
}

func (p *ProxyBlocklistsLoad) Terminate() error {
	return nil
}
