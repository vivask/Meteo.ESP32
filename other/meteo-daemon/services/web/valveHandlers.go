package web

import "net/http"

func (web *Web) valveHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "valve.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}
