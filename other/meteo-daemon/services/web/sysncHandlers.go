package web

import (
	"context"
	"net/http"
)

func (web *Web) tablesHandler(w http.ResponseWriter, r *http.Request) {
	keys, err := web.repo.GetAllSyncTables(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "tables.page.html", &keys, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) syncHandler(w http.ResponseWriter, r *http.Request) {
	keys, err := web.repo.GetAllSyncTables(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "sync.page.html", &keys, false)
	go web.metrics.ResponceCode(http.StatusOK)
}
