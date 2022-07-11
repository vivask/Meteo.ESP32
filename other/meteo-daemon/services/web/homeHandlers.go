package web

import (
	"context"
	"net/http"
	"strconv"

	"github.com/gorilla/mux"
)

func (web *Web) homeHandler(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		web.notFound(w)
		return
	}
	web.render(w, r, "home.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)

}

func (web *Web) getBmx280Handler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetBmx280(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getMics6814Handler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetMics6814(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getRadsensHandler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetRadsens(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getDs18b20Handler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetDs18b20(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getZe08ch2oHandler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetZe08ch2o(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getSettingsHandler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetSettings(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getEsp32DateTimeHandler(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetEsp32DateTime(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) checkBmx280Tempr(w http.ResponseWriter, r *http.Request) {
	err := web.repo.CheckBmx280Tempr(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) checkDs18b20(w http.ResponseWriter, r *http.Request) {
	err := web.repo.CheckDs18b20(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) checkZe08ch2o(w http.ResponseWriter, r *http.Request) {
	err := web.repo.CheckZe08ch2o(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) checkRadsensDyn(w http.ResponseWriter, r *http.Request) {
	err := web.repo.CheckRadsensDyn(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) checkRadsensStat(w http.ResponseWriter, r *http.Request) {
	err := web.repo.CheckRadsensStat(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) setRadsensHV(w http.ResponseWriter, r *http.Request) {
	istate, err := strconv.Atoi(mux.Vars(r)["hv"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	state := true
	if istate == 0 {
		state = false
	}
	err = web.repo.SetRadsensHV(context.Background(), state)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
