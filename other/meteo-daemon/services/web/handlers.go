package web

import (
	"net/http"
)

func (web *Web) Metrics(w http.ResponseWriter, req *http.Request) {
	web.metrics.Handler().ServeHTTP(w, req)
}

func (web *Web) signupHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "signup.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) loginHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "login.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}
