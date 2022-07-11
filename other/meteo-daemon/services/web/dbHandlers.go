package web

import (
	"context"
	"fmt"
	"meteo-daemon/services/data"
	"net/http"

	"github.com/gorilla/mux"
)

func (web *Web) addSyncTable(w http.ResponseWriter, r *http.Request) {
	var table data.SyncTables
	err := FromJSON(&table, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}

	err = web.repo.AddSyncTable(context.Background(), table)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) editSyncTable(w http.ResponseWriter, r *http.Request) {
	var table data.SyncTables
	err := FromJSON(&table, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.EditSyncTable(context.Background(), table, mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delSyncTable(w http.ResponseWriter, r *http.Request) {
	id := mux.Vars(r)["id"]
	err := web.repo.DelSyncTable(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getSyncTable(w http.ResponseWriter, r *http.Request) {
	id := mux.Vars(r)["id"]
	table, err := web.repo.GetSyncTable(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	ToJSON(table, w)
}

func (web *Web) execRaw(w http.ResponseWriter, r *http.Request) {
	var callback data.Callback
	err := FromJSON(&callback, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ExecRaw(context.Background(), callback)
	if err != nil {
		web.logger.Error(err)
		web.logger.Info(callback.Query)
		web.logger.Info(callback.Params...)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceHomezones(w http.ResponseWriter, r *http.Request) {
	var hosts []data.Homezone
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceHomezones(context.Background(), hosts)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceToVpnManual(w http.ResponseWriter, r *http.Request) {
	var hosts []data.ToVpnManual
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceToVpnManual(context.Background(), hosts)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceToVpnAuto(w http.ResponseWriter, r *http.Request) {
	var hosts []data.ToVpnAuto
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceToVpnAuto(context.Background(), hosts)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceToVpnIgnore(w http.ResponseWriter, r *http.Request) {
	var hosts []data.ToVpnIgnore
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceToVpnIgnore(context.Background(), hosts)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceSshKeys(w http.ResponseWriter, r *http.Request) {
	var hosts []data.SSHKeys
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceSshKeys(context.Background(), hosts)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceGitKeys(w http.ResponseWriter, r *http.Request) {
	var keys []data.GitKeys
	err := FromJSON(&keys, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceGitKeys(context.Background(), keys)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceGitUsers(w http.ResponseWriter, r *http.Request) {
	var users []data.GitUsers
	err := FromJSON(&users, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceGitUsers(context.Background(), users)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceTasks(w http.ResponseWriter, r *http.Request) {
	var tasks []data.Tasks
	err := FromJSON(&tasks, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceTasks(context.Background(), tasks)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceJobs(w http.ResponseWriter, r *http.Request) {
	var jobs []data.Jobs
	err := FromJSON(&jobs, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceJobs(context.Background(), jobs)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectSource(source, url string) (bool, error) {
	if (source == "master" && web.lead.IsMaster()) || (source == "slave" && !web.lead.IsMaster()) {
		web.logger.Infof("Reached: %s, master: %v", source, web.lead.IsMaster())
		return true, nil
	}

	if (source == "slave" && web.lead.IsMaster()) || (source == "master" && !web.lead.IsMaster()) {
		_, err := web.cli.PostExt(url, nil)
		if err != nil {
			return false, fmt.Errorf("replace error: %w", err)
		}
		web.logger.Infof("Jump: %s, master: %v, url: %s", source, web.lead.IsMaster(), url)
		return false, nil
	}

	return false, fmt.Errorf("unknown source")
}

func (web *Web) selectSyncSourceSyncBmx280(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/sync/bmx280")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.SyncBmx280(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceBmx280(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/bmx280")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtBmx280(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectSyncSourceSyncDs18b20(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/sync/ds18b20")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.SyncDs18b20(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceDs18b20(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/ds18b20")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtDs18b20(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectSyncSourceSyncZe08ch2o(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/sync/ze08ch2o")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.SyncZe08ch2o(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceZe08ch2o(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/ze08ch2o")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtZe08ch2o(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectSyncSourceSyncRadsens(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/sync/radsens")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.SyncRadsens(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceRadsens(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/radsens")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtRadsens(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceToVpnManual(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/tovpn_manuals")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtToVpnManual(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceToVpnAuto(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/tovpn_autos")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtToVpnAuto(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceToVpnIgnore(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/tovpn_ignores")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtToVpnIgnore(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceHomezones(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/homezones")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtHomeZones(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceSshKeys(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/ssh_keys")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtSshKeys(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceGitKeys(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/git_keys")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtGitKeys(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceGitUsers(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/git_users")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtGitUsers(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceTasks(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/tasks")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtTasks(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) selectReplaceSourceJobs(w http.ResponseWriter, r *http.Request) {
	source := mux.Vars(r)["source"]

	reached, err := web.selectSource(source, "/verify/db/rest/replace/jobs")
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	if reached {
		err = web.repo.ReplaceExtJobs(context.Background())
		if err != nil {
			web.logger.Error(err)
		}
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) MiddlewareValidateSync(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		w.Header().Set("Content-Type", "application/json")

		if !web.lead.IsAliveRemote() {
			errRemoteNotAvailable := fmt.Errorf("%s server not available", web.lead.Other())
			web.logger.Error(errRemoteNotAvailable)
			w.WriteHeader(http.StatusBadRequest)
			ToJSON(errRemoteNotAvailable, w)
			return
		}

		// call the next handler
		next.ServeHTTP(w, r)
	})
}
