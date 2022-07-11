package web

import (
	"encoding/json"
	"io"
	"net/http"
	"strconv"
)

type Response struct {
	Status string `json:"status"`
	Code   int    `json:"code"`
	Error  string `json:"error"`
}

// ToJSON serializes the given interface into a string based JSON format
func ToJSON(i interface{}, w io.Writer) error {
	e := json.NewEncoder(w)
	return e.Encode(i)
}

// FromJSON deserializes the object from JSON string
// given in the io.Reader to the given interface
func FromJSON(i interface{}, r io.Reader) error {
	d := json.NewDecoder(r)
	return d.Decode(i)
}

// GenericResponse is the format of our response
func GenericResponse(err error) (r *Response) {
	resp := Response{
		Status: http.StatusText(http.StatusOK),
		Code:   200,
	}
	if err != nil {
		resp.Status = http.StatusText(http.StatusInternalServerError)
		resp.Code = 500
		resp.Error = err.Error()
	}
	return &resp
}

func StringToUint32(s string) (uint32, error) {
	u, err := strconv.ParseUint(s, 10, 32)
	if err != nil {
		return 0, err
	}
	return uint32(u), nil
}
