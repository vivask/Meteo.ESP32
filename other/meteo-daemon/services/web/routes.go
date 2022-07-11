package web

import (
	"fmt"
	"html/template"
	"net/http"
	"path/filepath"
	"runtime/debug"
	s "strings"
	"time"

	"github.com/gorilla/mux"
)

func (web *Web) routes() *mux.Router {
	r := mux.NewRouter()

	refAuth := r.PathPrefix("/auth").Subrouter()
	refAuth.HandleFunc("/signup", web.auth.Signup)
	refAuth.HandleFunc("/login", web.auth.Login)
	refAuth.Use(web.auth.MiddlewareValidateUser)

	refRoot := r.PathPrefix("/").Subrouter()
	refRoot.PathPrefix("/static/").Handler(http.StripPrefix("/static/", http.FileServer(http.Dir("./ui/static/"))))
	refRoot.HandleFunc("/signup", web.signupHandler)
	refRoot.HandleFunc("/login", web.loginHandler)
	refRoot.HandleFunc("/", web.homeHandler)
	refRoot.HandleFunc("/bme280", web.bme280Handler)
	refRoot.HandleFunc("/radsens", web.radsensHandler)
	refRoot.HandleFunc("/ze08ch2o", web.ze08ch2oHandler)
	refRoot.HandleFunc("/ds18b20", web.ds18b20Handler)
	refRoot.HandleFunc("/mics6814", web.mics6814Handler)
	refRoot.HandleFunc("/valve", web.valveHandler)
	refRoot.HandleFunc("/upload", web.uploadHandler)
	refRoot.HandleFunc("/esp32", web.esp32Handler)
	refRoot.HandleFunc("/metrics", web.Metrics)

	refVerify := r.PathPrefix("/verify").Subrouter()
	refVerify.PathPrefix("/static/").Handler(http.StripPrefix("/verify/static/", http.FileServer(http.Dir("./ui/static/"))))
	refVerify.HandleFunc("/controller", web.controllerHandler)
	refVerify.HandleFunc("/proxy", web.proxyHandler)
	refVerify.HandleFunc("/homezone", web.homezoneHandler)
	refVerify.HandleFunc("/vpnauto", web.vpnautoHandler)
	refVerify.HandleFunc("/vpnmanual", web.vpnmanualHandler)
	refVerify.HandleFunc("/vpnignore", web.vpnignoreHandler)
	refVerify.HandleFunc("/settings", web.settingsHandler)
	refVerify.HandleFunc("/loging", web.logingHandler)
	refVerify.HandleFunc("/ssh", web.sshHandler)
	refVerify.HandleFunc("/knowhosts", web.knowhostsHandler)
	refVerify.HandleFunc("/gitkeys", web.gitKeysHandler)
	refVerify.HandleFunc("/gitusers", web.gitUsersHandler)
	refVerify.HandleFunc("/schedule", web.sheduleHandler)
	refVerify.HandleFunc("/tasks", web.tasksHandler)
	refVerify.HandleFunc("/cron", web.cronHandler)
	refVerify.HandleFunc("/tables", web.tablesHandler)
	refVerify.HandleFunc("/sync", web.syncHandler)
	//refVerify.Use(web.auth.MiddlewareValidateAccessToken)

	uiHome := r.PathPrefix("/ui/home").Subrouter()
	uiHome.Path("/bme280").Methods(http.MethodGet).HandlerFunc(web.getBmx280Handler)
	uiHome.Path("/mics6814").Methods(http.MethodGet).HandlerFunc(web.getMics6814Handler)
	uiHome.Path("/radsens").Methods(http.MethodGet).HandlerFunc(web.getRadsensHandler)
	uiHome.Path("/ds18b20").Methods(http.MethodGet).HandlerFunc(web.getDs18b20Handler)
	uiHome.Path("/ze08ch2o").Methods(http.MethodGet).HandlerFunc(web.getZe08ch2oHandler)
	uiHome.Path("/settings").Methods(http.MethodGet).HandlerFunc(web.getSettingsHandler)
	uiHome.Path("/esp32datetime").Methods(http.MethodGet).HandlerFunc(web.getEsp32DateTimeHandler)
	uiHome.Path("/bme280").Methods(http.MethodPut).HandlerFunc(web.checkBmx280Tempr)
	uiHome.Path("/ds18b20").Methods(http.MethodPut).HandlerFunc(web.checkDs18b20)
	uiHome.Path("/radstat").Methods(http.MethodPut).HandlerFunc(web.checkRadsensStat)
	uiHome.Path("/raddyn").Methods(http.MethodPut).HandlerFunc(web.checkRadsensDyn)
	uiHome.Path("/radhvset/{hv}").Methods(http.MethodPut).HandlerFunc(web.setRadsensHV)
	uiHome.Path("/ze08ch2o").Methods(http.MethodPut).HandlerFunc(web.checkZe08ch2o)

	uiBmx280 := r.PathPrefix("/ui/bmx280").Subrouter()
	uiBmx280.Path("/day/avg").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerDayAvg)
	uiBmx280.Path("/day/min").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerDayMin)
	uiBmx280.Path("/day/max").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerDayMax)
	uiBmx280.Path("/week/avg").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerWeekAvg)
	uiBmx280.Path("/week/min").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerWeekMin)
	uiBmx280.Path("/week/max").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerWeekMax)
	uiBmx280.Path("/month/avg").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerMonthAvg)
	uiBmx280.Path("/month/min").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerMonthMin)
	uiBmx280.Path("/month/max").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerMonthMax)
	uiBmx280.Path("/year/avg").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerYearAvg)
	uiBmx280.Path("/year/min").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerYearMin)
	uiBmx280.Path("/year/max").Methods(http.MethodPost).HandlerFunc(web.getBmx280PerYearMax)

	uiRadsens := r.PathPrefix("/ui/radsens").Subrouter()
	uiRadsens.Path("/day").Methods(http.MethodPost).HandlerFunc(web.getRadsensPerDay)
	uiRadsens.Path("/week").Methods(http.MethodPost).HandlerFunc(web.getRadsensPerWeek)
	uiRadsens.Path("/month").Methods(http.MethodPost).HandlerFunc(web.getRadsensPerMonth)
	uiRadsens.Path("/year").Methods(http.MethodPost).HandlerFunc(web.getRadsensPerYear)

	uiDs18b20 := r.PathPrefix("/ui/ds18b20").Subrouter()
	uiDs18b20.Path("/day").Methods(http.MethodPost).HandlerFunc(web.getDs18b20PerDay)
	uiDs18b20.Path("/week").Methods(http.MethodPost).HandlerFunc(web.getDs18b20PerWeek)
	uiDs18b20.Path("/month").Methods(http.MethodPost).HandlerFunc(web.getDs18b20PerMonth)
	uiDs18b20.Path("/year").Methods(http.MethodPost).HandlerFunc(web.getDs18b20PerYear)

	uiZe08ch2o := r.PathPrefix("/ui/ze08ch2o").Subrouter()
	uiZe08ch2o.Path("/day").Methods(http.MethodPost).HandlerFunc(web.getZe08ch2oPerDay)
	uiZe08ch2o.Path("/week").Methods(http.MethodPost).HandlerFunc(web.getZe08ch2oPerWeek)
	uiZe08ch2o.Path("/month").Methods(http.MethodPost).HandlerFunc(web.getZe08ch2oPerMonth)
	uiZe08ch2o.Path("/year").Methods(http.MethodPost).HandlerFunc(web.getZe08ch2oPerYear)

	uiLoging := r.PathPrefix("/verify/loging").Subrouter()
	uiLoging.Path("/get").Methods(http.MethodPost).HandlerFunc(web.getLoging)
	uiLoging.Path("/clear").Methods(http.MethodPut).HandlerFunc(web.journalClear)

	uiSetting := r.PathPrefix("/verify/settings").Subrouter()
	uiSetting.Path("/get").Methods(http.MethodGet).HandlerFunc(web.getSettings)
	uiSetting.Path("/set").Methods(http.MethodPost).HandlerFunc(web.setSettings)

	uiController := r.PathPrefix("/verify/controller").Subrouter()
	uiController.Path("/upgrade/{file}").Methods(http.MethodPut).HandlerFunc(web.startUpgradeEsp32)
	uiController.Path("/upgrade/status").Methods(http.MethodGet).HandlerFunc(web.getUpgradeStatus)
	uiController.Path("/upgrade/terminate").Methods(http.MethodPut).HandlerFunc(web.terminateUpgrade)
	uiController.Path("/ap").Methods(http.MethodPut).HandlerFunc(web.startAccesPointMode)
	uiController.Path("/ap/status").Methods(http.MethodGet).HandlerFunc(web.getStatusAccesPoint)
	uiController.Path("/reboot").Methods(http.MethodPut).HandlerFunc(web.startRebootEsp32)
	uiController.Path("/reboot/status").Methods(http.MethodGet).HandlerFunc(web.getRebootStatus)

	uiSecure := r.PathPrefix("/verify/secure").Subrouter()
	uiSecure.Path("/ssh/add").Methods(http.MethodPost).HandlerFunc(web.addSshKey)
	uiSecure.Path("/ssh/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delSshKey)
	uiSecure.Path("/knowhosts/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delKnowhost)
	uiSecure.Path("/gitkey/add").Methods(http.MethodPost).HandlerFunc(web.addGitKey)
	uiSecure.Path("/gitkey/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delGitKey)
	uiSecure.Path("/gituser/add").Methods(http.MethodPost).HandlerFunc(web.addGitUser)
	uiSecure.Path("/gituser/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delGitUser)

	uiSchedule := r.PathPrefix("/verify/schedule").Subrouter()
	uiSchedule.Path("/job/add").Methods(http.MethodPost).HandlerFunc(web.addJob)
	uiSchedule.Path("/job/edit/{id}").Methods(http.MethodPut).HandlerFunc(web.editJob)
	uiSchedule.Path("/job/activate/{id}").Methods(http.MethodPut).HandlerFunc(web.activateJob)
	uiSchedule.Path("/job/run/{id}").Methods(http.MethodPut).HandlerFunc(web.runJob)
	uiSchedule.Path("/job/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delJob)
	uiSchedule.Path("/job/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getJob)
	uiSchedule.Path("/job/params/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getJobParams)
	uiSchedule.Path("/task/add").Methods(http.MethodPost).HandlerFunc(web.addTask)
	uiSchedule.Path("/task/edit/{id}").Methods(http.MethodPut).HandlerFunc(web.editTask)
	uiSchedule.Path("/task/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delTask)
	uiSchedule.Path("/task/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getTask)
	uiSchedule.Path("/task/params/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getTaskParams)

	restSchedule := r.PathPrefix("/verify/schedule/rest").Subrouter()
	restSchedule.Path("/task/implemented/{id}").Methods(http.MethodGet).HandlerFunc(web.isImplemented)
	restSchedule.Path("/job/update/{id}").Methods(http.MethodPut).HandlerFunc(web.jobUpdate)
	restSchedule.Path("/job/create/{id}").Methods(http.MethodPut).HandlerFunc(web.jobCreate)
	restSchedule.Path("/job/run/{id}").Methods(http.MethodPut).HandlerFunc(web.jobRun)
	restSchedule.Path("/job/remove/{id}").Methods(http.MethodDelete).HandlerFunc(web.jobRemove)
	restSchedule.Path("/job/reload").Methods(http.MethodPut).HandlerFunc(web.reloadJobs)
	restSchedule.Use(web.MiddlewareValidateShedule)

	restTelegram := r.PathPrefix("/verify/telegram/rest").Subrouter()
	restTelegram.Path("/message").Methods(http.MethodPost).HandlerFunc(web.telegramSend)
	restSchedule.Use(web.MiddlewareValidateTelegram)

	uiProxy := r.PathPrefix("/verify/proxy").Subrouter()
	uiProxy.Path("/manual/add").Methods(http.MethodPost).HandlerFunc(web.addManualToVpn)
	uiProxy.Path("/manual/edit/{id}").Methods(http.MethodPost).HandlerFunc(web.editManualToVpn)
	uiProxy.Path("/manual/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delManualToVpn)
	uiProxy.Path("/manual/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getManualToVpn)
	uiProxy.Path("/master/onoff").Methods(http.MethodPut).HandlerFunc(web.changeMasterState)
	uiProxy.Path("/slave/onoff").Methods(http.MethodPut).HandlerFunc(web.changeSlaveState)
	uiProxy.Path("/master/cache/onoff").Methods(http.MethodPut).HandlerFunc(web.changeMasterCache)
	uiProxy.Path("/slave/cache/onoff").Methods(http.MethodPut).HandlerFunc(web.changeSlaveCache)
	uiProxy.Path("/master/blklist/onoff").Methods(http.MethodPut).HandlerFunc(web.changeMasterBlk)
	uiProxy.Path("/slave/blklist/onoff").Methods(http.MethodPut).HandlerFunc(web.changeSlaveBlk)
	uiProxy.Path("/master/unlocker/onoff").Methods(http.MethodPut).HandlerFunc(web.changeMasterUnlocker)
	uiProxy.Path("/slave/unlocker/onoff").Methods(http.MethodPut).HandlerFunc(web.changeSlaveUnlocker)
	uiProxy.Path("/auto/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delAutoToVpn)
	uiProxy.Path("/ignore/add/{id}").Methods(http.MethodPut).HandlerFunc(web.addIgnoreToVpn)
	uiProxy.Path("/ignore/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delIgnoreToVpn)
	uiProxy.Path("/ignore/restore/{id}").Methods(http.MethodPut).HandlerFunc(web.restoreIgnoreToVpn)
	uiProxy.Path("/homehost/add").Methods(http.MethodPut).HandlerFunc(web.addHomeZoneHost)
	uiProxy.Path("/homehost/edit/{id}").Methods(http.MethodPost).HandlerFunc(web.editHomeZoneHost)
	uiProxy.Path("/homehost/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delHomeZoneHost)
	uiProxy.Path("/homehost/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getHomeZoneHost)

	restProxy := r.PathPrefix("/verify/proxy/rest").Subrouter()
	restProxy.Path("/state/get").Methods(http.MethodGet).HandlerFunc(web.getState)
	restProxy.Path("/sync").Methods(http.MethodPost).HandlerFunc(web.syncProxyZones)
	restProxy.Path("/sync").Methods(http.MethodPut).HandlerFunc(web.reloadProxyZones)
	restProxy.Path("/onoff").Methods(http.MethodPut).HandlerFunc(web.changeState)
	restProxy.Path("/cache/onoff").Methods(http.MethodPut).HandlerFunc(web.changeCache)
	restProxy.Path("/blklist/onoff").Methods(http.MethodPut).HandlerFunc(web.changeBlk)
	restProxy.Path("/unlocker/onoff").Methods(http.MethodPut).HandlerFunc(web.changeUnlocker)
	restProxy.Path("/blklist/update").Methods(http.MethodPost).HandlerFunc(web.blklistUpdate)
	restProxy.Path("/arp/ping/{ip}").Methods(http.MethodGet).HandlerFunc(web.arpPing)
	restProxy.Path("/homehost/add").Methods(http.MethodPut).HandlerFunc(web.addHost)
	restProxy.Path("/homehost/edit/{id}").Methods(http.MethodPost).HandlerFunc(web.editHost)
	restProxy.Path("/homehost/del/{name}").Methods(http.MethodDelete).HandlerFunc(web.delHost)
	restSchedule.Use(web.MiddlewareValidateProxy)

	restMikrotiks := r.PathPrefix("/verify/mikrotiks/rest").Subrouter()
	restMikrotiks.Path("/sync").Methods(http.MethodPost).HandlerFunc(web.syncRouterZones)
	restMikrotiks.Path("/tovpn").Methods(http.MethodPut).HandlerFunc(web.putHostToVpn)
	restMikrotiks.Path("/rmvpn").Methods(http.MethodPut).HandlerFunc(web.removeHostFromVpn)
	restMikrotiks.Use(web.MiddlewareValidateMikrotiks)

	uiDBSync := r.PathPrefix("/verify/db").Subrouter()
	uiDBSync.Path("/table/add").Methods(http.MethodPost).HandlerFunc(web.addSyncTable)
	uiDBSync.Path("/table/edit/{id}").Methods(http.MethodPut).HandlerFunc(web.editSyncTable)
	uiDBSync.Path("/table/del/{id}").Methods(http.MethodDelete).HandlerFunc(web.delSyncTable)
	uiDBSync.Path("/table/get/{id}").Methods(http.MethodGet).HandlerFunc(web.getSyncTable)

	restDBSync := r.PathPrefix("/verify/db/rest").Subrouter()
	restDBSync.Path("/exec").Methods(http.MethodPost).HandlerFunc(web.execRaw)
	restDBSync.Path("/bmx280").Methods(http.MethodGet).HandlerFunc(web.getNotSyncBmx280)
	restDBSync.Path("/bmx280").Methods(http.MethodPost).HandlerFunc(web.addSyncBmx280)
	restDBSync.Path("/sync/bmx280").Methods(http.MethodPost).HandlerFunc(web.syncBmx280)
	restDBSync.Path("/replace/bmx280").Methods(http.MethodPost).HandlerFunc(web.replaceBmx280)
	restDBSync.Path("/lock/bmx280").Methods(http.MethodPost).HandlerFunc(web.lockBmx280)
	restDBSync.Path("/unlock/bmx280").Methods(http.MethodPost).HandlerFunc(web.unlockBmx280)
	restDBSync.Path("/ds18b20").Methods(http.MethodGet).HandlerFunc(web.getNotSyncDs18b20)
	restDBSync.Path("/ds18b20").Methods(http.MethodPost).HandlerFunc(web.addSyncDs18b20)
	restDBSync.Path("/sync/ds18b20").Methods(http.MethodPost).HandlerFunc(web.syncDs18b20)
	restDBSync.Path("/replace/ds18b20").Methods(http.MethodPost).HandlerFunc(web.replaceDs18b20)
	restDBSync.Path("/lock/ds18b20").Methods(http.MethodPost).HandlerFunc(web.lockDs18b20)
	restDBSync.Path("/unlock/ds18b20").Methods(http.MethodPost).HandlerFunc(web.unlockDs18b20)
	restDBSync.Path("/ze08ch2o").Methods(http.MethodGet).HandlerFunc(web.getNotSyncZe08ch2o)
	restDBSync.Path("/ze08ch2o").Methods(http.MethodPost).HandlerFunc(web.addSyncZe08ch2o)
	restDBSync.Path("/sync/ze08ch2o").Methods(http.MethodPost).HandlerFunc(web.syncZe08ch2o)
	restDBSync.Path("/replace/ze08ch2o").Methods(http.MethodPost).HandlerFunc(web.replaceZe08ch2o)
	restDBSync.Path("/lock/ze08ch2o").Methods(http.MethodPost).HandlerFunc(web.lockZe08ch2o)
	restDBSync.Path("/unlock/ze08ch2o").Methods(http.MethodPost).HandlerFunc(web.unlockZe08ch2o)
	restDBSync.Path("/radsens").Methods(http.MethodGet).HandlerFunc(web.getNotSyncRadsens)
	restDBSync.Path("/radsens").Methods(http.MethodPost).HandlerFunc(web.addSyncRadsens)
	restDBSync.Path("/sync/radsens").Methods(http.MethodPost).HandlerFunc(web.syncRadsens)
	restDBSync.Path("/replace/radsens").Methods(http.MethodPost).HandlerFunc(web.replaceRadsens)
	restDBSync.Path("/lock/radsens").Methods(http.MethodPost).HandlerFunc(web.lockRadsens)
	restDBSync.Path("/unlock/radsens").Methods(http.MethodPost).HandlerFunc(web.unlockRadsens)
	restDBSync.Path("/replace/tovpn_manuals").Methods(http.MethodPost).HandlerFunc(web.replaceToVpnManual)
	restDBSync.Path("/replace/tovpn_autos").Methods(http.MethodPost).HandlerFunc(web.replaceToVpnAuto)
	restDBSync.Path("/replace/tovpn_ignores").Methods(http.MethodPost).HandlerFunc(web.replaceToVpnIgnore)
	restDBSync.Path("/replace/homezones").Methods(http.MethodPost).HandlerFunc(web.replaceHomezones)
	restDBSync.Path("/replace/ssh_keys").Methods(http.MethodPost).HandlerFunc(web.replaceSshKeys)
	restDBSync.Path("/replace/git_keys").Methods(http.MethodPost).HandlerFunc(web.replaceGitKeys)
	restDBSync.Path("/replace/git_users").Methods(http.MethodPost).HandlerFunc(web.replaceGitUsers)
	restDBSync.Path("/replace/tasks").Methods(http.MethodPost).HandlerFunc(web.replaceTasks)
	restDBSync.Path("/replace/jobs").Methods(http.MethodPost).HandlerFunc(web.replaceJobs)
	//select source master or slave
	restDBSync.Path("/sync/bmx280/{source}").Methods(http.MethodPut).HandlerFunc(web.selectSyncSourceSyncBmx280)
	restDBSync.Path("/replace/bmx280/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceBmx280)
	restDBSync.Path("/sync/ds18b20/{source}").Methods(http.MethodPut).HandlerFunc(web.selectSyncSourceSyncDs18b20)
	restDBSync.Path("/replace/ds18b20/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceDs18b20)
	restDBSync.Path("/sync/ze08ch2o/{source}").Methods(http.MethodPut).HandlerFunc(web.selectSyncSourceSyncZe08ch2o)
	restDBSync.Path("/replace/ze08ch2o/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceZe08ch2o)
	restDBSync.Path("/sync/radsens/{source}").Methods(http.MethodPut).HandlerFunc(web.selectSyncSourceSyncRadsens)
	restDBSync.Path("/replace/radsens/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceRadsens)
	restDBSync.Path("/replace/tovpn_manuals/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceToVpnManual)
	restDBSync.Path("/replace/tovpn_autos/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceToVpnAuto)
	restDBSync.Path("/replace/tovpn_ignores/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceToVpnIgnore)
	restDBSync.Path("/replace/homezones/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceHomezones)
	restDBSync.Path("/replace/ssh_keys/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceSshKeys)
	restDBSync.Path("/replace/git_keys/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceGitKeys)
	restDBSync.Path("/replace/git_users/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceGitUsers)
	restDBSync.Path("/replace/tasks/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceTasks)
	restDBSync.Path("/replace/jobs/{source}").Methods(http.MethodPut).HandlerFunc(web.selectReplaceSourceJobs)
	restDBSync.Use(web.MiddlewareValidateSync)

	return r
}

func (web *Web) serverError(w http.ResponseWriter, err error) {
	trace := fmt.Sprintf("%s\n%s", err.Error(), debug.Stack())
	web.logger.Trace(trace)

	http.Error(w, http.StatusText(http.StatusInternalServerError), http.StatusInternalServerError)
}

func (web *Web) clientError(w http.ResponseWriter, status int) {
	http.Error(w, http.StatusText(status), status)
}

func (web *Web) notFound(w http.ResponseWriter) {
	web.clientError(w, http.StatusNotFound)
}

func (web *Web) render(w http.ResponseWriter, r *http.Request, name string, data interface{}, reload bool) {
	/*if reload {
		err := web.reloadTemplate(web.templateCache, name)
		if err != nil {
			web.serverError(w, err)
			return
		}
	}*/
	ts, ok := web.templateCache[name]
	if !ok {
		web.serverError(w, fmt.Errorf("template %s not found", name))
		return
	}
	tname := s.Split(name, ".")[0]
	err := ts.ExecuteTemplate(w, tname, data)
	if err != nil {
		web.logger.Error(err)
		web.serverError(w, err)
	}
}

func (web *Web) reloadTemplate(cache map[string]*template.Template, html string) error {
	page := fmt.Sprintf("%s/%s", web.config.UiPath, html)
	name := filepath.Base(page)
	ts, err := template.New(name).Funcs(template.FuncMap{
		"DataFmtNil":     web.DataFmtNil,
		"DateFmtLong":    web.DateFmtLong,
		"DateFmtSimple":  web.DateFmtSimple,
		"DateFmtShort":   web.DateFmtShort,
		"SubStr":         web.SubStr,
		"StrToLower":     web.StrToLower,
		"ByteToLowerStr": web.ByteToLowerStr,
		"ByteToStr":      web.ByteToStr,
		"CaseDayStr":     web.CaseDayStr,
		"GetSecSuffix":   web.GetSecSuffix,
		"GetHourSuffix":  web.GetHourSuffix,
		"GetPeriod":      web.GetPeriod,
		"NotZeroTime":    web.NotZeroTime,
	}).ParseFiles(page)
	if err != nil {
		return err
	}
	ts, err = ts.ParseGlob(filepath.Join(web.config.UiPath, "*.partial.html"))
	if err != nil {
		return err
	}
	cache[name] = ts
	return nil
}

func (web *Web) newTemplateCache(dir string) (map[string]*template.Template, error) {

	cache := map[string]*template.Template{}
	pages, err := filepath.Glob(filepath.Join(dir, "*.page.html"))
	if err != nil {
		return nil, err
	}
	for _, page := range pages {
		name := filepath.Base(page)
		ts, err := template.New(name).Funcs(template.FuncMap{
			"DataFmtNil":     web.DataFmtNil,
			"DateFmtLong":    web.DateFmtLong,
			"DateFmtSimple":  web.DateFmtSimple,
			"DateFmtShort":   web.DateFmtShort,
			"SubStr":         web.SubStr,
			"StrToLower":     web.StrToLower,
			"ByteToLowerStr": web.ByteToLowerStr,
			"ByteToStr":      web.ByteToStr,
			"CaseDayStr":     web.CaseDayStr,
			"GetSecSuffix":   web.GetSecSuffix,
			"GetHourSuffix":  web.GetHourSuffix,
			"GetPeriod":      web.GetPeriod,
			"NotZeroTime":    web.NotZeroTime,
		}).ParseFiles(page)
		if err != nil {
			return nil, err
		}
		ts, err = ts.ParseGlob(filepath.Join(dir, "*.partial.html"))
		if err != nil {
			return nil, err
		}

		cache[name] = ts
	}
	return cache, nil
}

func (web *Web) DataFmtNil(t interface{}) string {
	if t == nil {
		return ""
	} else {
		return t.(string)
	}
}

func (web *Web) DateFmtLong(t time.Time) string {
	return t.Format("Jan 02, 2006 15:04:05")
}

func (web *Web) DateFmtSimple(t time.Time) string {
	return t.Format("2006-01-02 15:04:05")
}

func (web *Web) DateFmtShort(t time.Time) string {
	return t.Format("Jan 02, 2006")
}

func (web *Web) SubStr(str string, start, length int) string {
	if str == "" {
		return ""
	}
	end := start + length
	if length == -1 {
		end = len(str)
	}
	if len(str) < end {
		return str
	}
	return str[start:end]
}

func (web *Web) StrToLower(str string) string {
	return s.ToLower(str)
}

func (web *Web) ByteToLowerStr(b []byte) string {
	return s.ToLower(string(b))
}

func (web *Web) ByteToStr(b []byte) string {
	return string(b)
}

func (web *Web) CaseDayStr(idx int) string {
	switch idx {
	case 1:
		return "понедельник"
	case 2:
		return "вторник"
	case 3:
		return "среду"
	case 4:
		return "четверг"
	case 5:
		return "пятницу"
	case 6:
		return "субботу"
	case 7:
		return "воскресенье"
	default:
		return fmt.Sprintf("Не корректный номер дня недели: %d", idx)
	}
}

func (web *Web) GetSecSuffix(key string, val int) string {
	if key == "one" {
		return "Выполнить "
	}
	if val <= 1 || (val > 20 && val%10 == 1) {
		return "Повторять каждую "
	} else {
		return "Повторять каждые "
	}
}

func (web *Web) GetHourSuffix(key string, val int) string {
	if val <= 1 || (val > 20 && val%10 == 1) {
		return "Повторять каждый "
	} else {
		return "Повторять каждые "
	}
}

func (web *Web) GetPeriod(key string, val int) string {
	p1 := map[string]string{"one": "единоразово", "sec": "секунд", "min": "минут", "hour": "часов", "day": "дней", "week": "недель", "day_of_week": "день недели", "month": "месяцев"}
	p2 := map[string]string{"one": "единоразово", "sec": "секунду", "min": "минуту", "hour": "час", "day": "день", "week": "неделю", "day_of_week": "день недели", "month": "месяц"}
	p3 := map[string]string{"one": "единоразово", "sec": "секунды", "min": "минуты", "hour": "часа", "day": "дня", "week": "недели", "day_of_week": "день недели", "month": "месяца"}
	if key == "one" {
		return p1[key]
	}
	if val == 1 {
		return p2[key]
	} else if val > 1 && val < 5 {
		return p3[key]
	} else {
		return p1[key]
	}
}

func (web *Web) NotZeroTime(t time.Time) bool {
	return !t.IsZero()
}
