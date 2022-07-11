package web

import (
	"context"
	"net/http"
)

func (web *Web) settingsHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "settings.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) getSettings(w http.ResponseWriter, r *http.Request) {
	s, err := web.repo.GetSettings(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&s, w)
}

func (web *Web) setSettings(w http.ResponseWriter, r *http.Request) {
	s, err := web.repo.GetSettings(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	err = FromJSON(s, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.repo.SetSettings(context.Background(), s)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
