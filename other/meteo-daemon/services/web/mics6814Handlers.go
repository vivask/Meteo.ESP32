package web

import "net/http"

func (web *Web) mics6814Handler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "mics6814.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}
