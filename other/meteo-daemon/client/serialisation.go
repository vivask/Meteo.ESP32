package client

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"net/http"
)

// GenericResponse is the format of our response
type GenericResponse struct {
	Status string `json:"status"`
	Code   int    `json:"code"`
	Error  string `json:"error"`
}

func FromJSON(r *http.Response) (body []byte, err error) {
	var gr GenericResponse
	body, err = ioutil.ReadAll(r.Body)
	if err != nil {
		return
	}
	err = json.Unmarshal(body, &gr)
	if err != nil {
		return body, nil
	}
	if gr.Code == 500 {
		return nil, fmt.Errorf(gr.Error)
	}
	return body, nil
}
