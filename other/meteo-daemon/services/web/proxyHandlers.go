package web

import (
	"context"
	"encoding/json"
	"fmt"
	"meteo-daemon/services/data"
	"net/http"

	"github.com/gorilla/mux"
	"github.com/mostlygeek/arp"
)

func (web *Web) proxyHandler(w http.ResponseWriter, r *http.Request) {
	var data struct {
		Master data.ProxyState
		Slave  data.ProxyState
	}

	intBody, err := web.cli.GetInt("/verify/proxy/rest/state/get")
	if err != nil {
		web.logger.Warnf("Internal Proxy server not responding: %v", err)
	}
	extBody, err := web.cli.GetExt("/verify/proxy/rest/state/get")
	if err != nil {
		web.logger.Warnf("Internal Proxy server not responding: %v", err)
	}
	if web.lead.IsMaster() {
		err = json.Unmarshal(intBody, &data.Master)
		if err != nil {
			web.logger.Error(err)
			return
		}
		err = json.Unmarshal(extBody, &data.Slave)
		if err != nil {
			web.logger.Error(err)
			return
		}
	} else {
		err = json.Unmarshal(extBody, &data.Master)
		if err != nil {
			web.logger.Error(err)
			return
		}
		err = json.Unmarshal(intBody, &data.Slave)
		if err != nil {
			web.logger.Error(err)
			return
		}
	}

	web.render(w, r, "proxy.page.html", &data, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) homezoneHandler(w http.ResponseWriter, r *http.Request) {
	hosts, err := web.repo.GetAllHomeZoneHosts(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "homezone.page.html", &hosts, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) vpnautoHandler(w http.ResponseWriter, r *http.Request) {
	hosts, err := web.repo.GetAllAutoToVpn(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "vpnauto.page.html", &hosts, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) vpnignoreHandler(w http.ResponseWriter, r *http.Request) {
	hosts, err := web.repo.GetAllIgnore(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "vpnignore.page.html", &hosts, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) vpnmanualHandler(w http.ResponseWriter, r *http.Request) {
	var data struct {
		Hosts []data.ToVpnManual
		Lists []data.AccesList
	}
	var err error
	data.Hosts, err = web.repo.GetAllManualToVpn(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	data.Lists, err = web.repo.GetAllAccessLists(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "vpnmanual.page.html", &data, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) addManualToVpn(w http.ResponseWriter, r *http.Request) {
	var host data.ToVpnManual
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.px.InsertManual(host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) editManualToVpn(w http.ResponseWriter, r *http.Request) {
	var host data.ToVpnManual
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	host.ID, err = StringToUint32(mux.Vars(r)["id"])
	err = web.repo.EditManualToVpn(context.Background(), host)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.px.UpdateManual(host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delManualToVpn(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.px.RemoveManual(id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getManualToVpn(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	jobs, err := web.repo.GetManualToVpn(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&jobs, w)
}

func (web *Web) changeMasterState(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutMaster("/verify/proxy/rest/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeSlaveState(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutSlave("/verify/proxy/rest/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeMasterCache(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutMaster("/verify/proxy/rest/cache/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeSlaveCache(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutSlave("/verify/proxy/rest/cache/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeMasterBlk(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutMaster("/verify/proxy/rest/blklist/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeSlaveBlk(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutSlave("/verify/proxy/rest/blklist/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeMasterUnlocker(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutMaster("/verify/proxy/rest/unlocker/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeSlaveUnlocker(w http.ResponseWriter, r *http.Request) {
	_, err := web.cli.PutSlave("/verify/proxy/rest/unlocker/onoff", nil)
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delAutoToVpn(w http.ResponseWriter, r *http.Request) {
	err := web.px.DeleteAutoHost(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) addIgnoreToVpn(w http.ResponseWriter, r *http.Request) {
	err := web.px.IgnoreAutoHost(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delIgnoreToVpn(w http.ResponseWriter, r *http.Request) {
	err := web.px.DeleteIgnore(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) restoreIgnoreToVpn(w http.ResponseWriter, r *http.Request) {
	err := web.px.RestoreIgnored(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getState(w http.ResponseWriter, r *http.Request) {
	state := &data.ProxyState{}
	if web.px.IsRun() {
		state = web.px.GetState()
	}
	ToJSON(state, w)
}

func (web *Web) syncProxyZones(w http.ResponseWriter, r *http.Request) {
	if web.px == nil {
		err := fmt.Errorf("service Proxy not available")
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	host := data.Homezone{}
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	web.px.SyncZones(host)
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) reloadProxyZones(w http.ResponseWriter, r *http.Request) {
	if web.px == nil {
		err := fmt.Errorf("service Proxy not available")
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	web.px.ReloadZones()
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) changeState(w http.ResponseWriter, r *http.Request) {
	var err error
	if web.px.IsRun() {
		err = web.px.Stop()
		if err == nil {
			web.logger.Info("Service [Proxy server] deactivated")
		}
	} else {
		err = web.px.Start()
		if err == nil {
			web.logger.Info("Service [Proxy server] activated")
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) changeCache(w http.ResponseWriter, r *http.Request) {
	web.px.ChangeCache()
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) changeBlk(w http.ResponseWriter, r *http.Request) {
	web.px.ChangeBlackList()
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) changeUnlocker(w http.ResponseWriter, r *http.Request) {
	web.px.ChangeUnlocker()
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) blklistUpdate(w http.ResponseWriter, r *http.Request) {
	hosts := map[string]struct{}{}
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}

	web.px.ReplaceBlocklist(hosts)
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) arpPing(w http.ResponseWriter, r *http.Request) {
	hosts, err := web.repo.GetAllHomeZoneHosts(context.Background())
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	ip := mux.Vars(r)["ip"]
	mac := arp.Search(ip)
	for i := range hosts {
		if hosts[i].IPv4 == ip {
			hosts[i].Mac = mac
		}
	}
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) addHomeZoneHost(w http.ResponseWriter, r *http.Request) {
	var host data.Homezone
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	mac := arp.Search(host.IPv4)
	if len(mac) > 0 {
		host.Mac = mac
	}

	err = web.repo.AddHomeZoneHost(context.Background(), host)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	web.px.AddHomeZoneHost(host.DomainName, host.IPv4)

	_, err = web.cli.PutExt("/verify/proxy/rest/homehost/add", host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) editHomeZoneHost(w http.ResponseWriter, r *http.Request) {
	var host data.Homezone
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	host.ID, err = StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	mac := arp.Search(host.IPv4)
	if len(mac) > 0 {
		host.Mac = mac
	}

	err = web.repo.EditHomeZoneHost(context.Background(), host)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	web.px.UpdateHomeZoneHost(host.DomainName, host.IPv4)

	url := fmt.Sprintf("/verify/proxy/rest/homehost/edit/%d", host.ID)
	_, err = web.cli.PostExt(url, host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delHomeZoneHost(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	host, err := web.repo.GetHomeZoneHost(context.Background(), id)
	err = web.repo.DelHomeZoneHost(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	web.px.RemoveHomeZoneHost(host.DomainName)

	url := fmt.Sprintf("/verify/proxy/rest/homehost/del/%s", host.DomainName)
	_, err = web.cli.DeleteExt(url)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getHomeZoneHost(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	host, err := web.repo.GetHomeZoneHost(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	ToJSON(host, w)
}

func (web *Web) addHost(w http.ResponseWriter, r *http.Request) {
	var host data.Homezone
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	web.px.AddHomeZoneHost(host.DomainName, host.IPv4)
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) editHost(w http.ResponseWriter, r *http.Request) {
	var host data.Homezone
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	web.px.UpdateHomeZoneHost(host.DomainName, host.IPv4)
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) delHost(w http.ResponseWriter, r *http.Request) {
	web.logger.Infof("delHost: %s", mux.Vars(r)["name"])
	web.px.RemoveHomeZoneHost(mux.Vars(r)["name"])
	ToJSON(GenericResponse(nil), w)
}

func (web *Web) MiddlewareValidateProxy(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		w.Header().Set("Content-Type", "application/json")

		if web.px == nil {
			errServiceNotAvailable := fmt.Errorf("service Proxy not available")
			web.logger.Error(errServiceNotAvailable)
			w.WriteHeader(http.StatusBadRequest)
			ToJSON(errServiceNotAvailable, w)
			return
		}

		// call the next handler
		next.ServeHTTP(w, r)
	})
}
