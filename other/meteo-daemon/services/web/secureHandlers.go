package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"

	"github.com/gorilla/mux"
)

func (web *Web) sshHandler(w http.ResponseWriter, r *http.Request) {
	keys, err := web.repo.GetAllSshKeys(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "ssh.page.html", &keys, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) knowhostsHandler(w http.ResponseWriter, r *http.Request) {
	knownhosts, err := web.repo.GetAllKnowhosts(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "knowhosts.page.html", &knownhosts, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) gitKeysHandler(w http.ResponseWriter, r *http.Request) {
	keys, err := web.repo.GetAllGitKeys(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "gitkeys.page.html", &keys, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) gitUsersHandler(w http.ResponseWriter, r *http.Request) {
	users, err := web.repo.GetAllGitUsers(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "gitusers.page.html", &users, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) addSshKey(w http.ResponseWriter, r *http.Request) {
	var key data.SSHKeys
	err := FromJSON(&key, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.AddSshKey(context.Background(), key)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delSshKey(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.DelSshKey(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delKnowhost(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.DelKnowhost(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) addGitKey(w http.ResponseWriter, r *http.Request) {
	var key data.GitKeys
	err := FromJSON(&key, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.AddGitKey(context.Background(), key)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delGitKey(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.DelGitKey(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) addGitUser(w http.ResponseWriter, r *http.Request) {
	var user data.GitUsers
	err := FromJSON(&user, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.AddGitUser(context.Background(), user)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delGitUser(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.DelGitUser(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
