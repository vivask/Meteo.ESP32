package web

import (
	"context"
	"net/http"

	"github.com/gorilla/mux"
)

func (web *Web) controllerHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "controller.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) startUpgradeEsp32(w http.ResponseWriter, r *http.Request) {
	fName := mux.Vars(r)["file"]
	err := web.repo.StartUpgradeEsp32(context.Background(), fName)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getUpgradeStatus(w http.ResponseWriter, r *http.Request) {
	s, err := web.repo.GetUpgradeStatus(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&s, w)
}

func (web *Web) terminateUpgrade(w http.ResponseWriter, r *http.Request) {
	err := web.repo.TerminateUpgrade(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) startAccesPointMode(w http.ResponseWriter, r *http.Request) {
	err := web.repo.StartAccesPointMode(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getStatusAccesPoint(w http.ResponseWriter, r *http.Request) {
	s, err := web.repo.GetStatusAccesPoint(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&s, w)
}

func (web *Web) startRebootEsp32(w http.ResponseWriter, r *http.Request) {
	err := web.repo.StartRebootEsp32(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getRebootStatus(w http.ResponseWriter, r *http.Request) {
	s, err := web.repo.GetRebootStatus(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&s, w)
}
