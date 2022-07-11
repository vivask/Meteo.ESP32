package client

import (
	"bytes"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"meteo-daemon/leader"
	"net"
	"net/http"
	URL "net/url"
	"time"

	"github.com/sirupsen/logrus"
)

const (
	INTERNAL = false
	EXTERNAL = true
)

const (
	POST   = true
	PUT    = false
	GET    = true
	DELETE = false
)

type Params struct {
	Logger     *logrus.Entry
	Lead       *leader.Leader
	Protocol   string
	LocalAddr  string
	LocalPort  int
	RemoteAddr string
	RemotePort int
	CA         string
	CrtClient  string
	KeyClient  string
}

type Client struct {
	logger   *logrus.Entry
	lead     *leader.Leader
	protocol string
	internal string
	external string
	cli      *http.Client
}

func New(p *Params) (*Client, error) {
	var client *http.Client
	var internal, external string
	switch p.Protocol {
	case "http":
		internal = fmt.Sprintf("http://%s:%d", p.LocalAddr, p.LocalPort)
		external = fmt.Sprintf("http://%s:%d", p.RemoteAddr, p.RemotePort)

		client = http.DefaultClient

	case "https":
		caCert, err := ioutil.ReadFile(p.CA)
		if err != nil {
			return nil, fmt.Errorf("error read CA: %w", err)
		}
		caCertPool := x509.NewCertPool()
		caCertPool.AppendCertsFromPEM(caCert)
		cert, err := tls.LoadX509KeyPair(p.CrtClient, p.KeyClient)
		if err != nil {
			p.Logger.Fatalf("server: loadkeys: %s", err)
		}

		internal = fmt.Sprintf("https://%s:%d", p.LocalAddr, p.LocalPort)
		external = fmt.Sprintf("https://%s:%d", p.RemoteAddr, p.RemotePort)

		client = &http.Client{
			Transport: &http.Transport{
				Proxy: http.ProxyFromEnvironment,
				DialContext: (&net.Dialer{
					Timeout:   30 * time.Second,
					KeepAlive: 30 * time.Second,
					DualStack: true,
				}).DialContext,
				ForceAttemptHTTP2:     true,
				MaxIdleConns:          100,
				IdleConnTimeout:       90 * time.Second,
				TLSHandshakeTimeout:   10 * time.Second,
				ExpectContinueTimeout: 1 * time.Second,
				TLSClientConfig: &tls.Config{
					RootCAs:            caCertPool,
					Certificates:       []tls.Certificate{cert},
					InsecureSkipVerify: true,
				},
			},
		}
	default:
		return nil, fmt.Errorf("unknown protocol: %s", p.Protocol)
	}

	return &Client{
		logger:   p.Logger,
		internal: internal,
		external: external,
		lead:     p.Lead,
		cli:      client,
	}, nil
}

func (c *Client) post(path string, r interface{}, post, ext bool) ([]byte, error) {
	jsonStr, err := json.Marshal(r)
	if err != nil {
		return nil, fmt.Errorf("Error JSON Marshal: %v", err)
	}
	var url string
	if ext {
		url = fmt.Sprintf("%s%s", c.external, path)
	} else {
		url = fmt.Sprintf("%s%s", c.internal, path)
	}

	method := http.MethodPost
	if !post {
		method = http.MethodPut
	}

	req, err := http.NewRequest(method, url, bytes.NewBuffer(jsonStr))
	if err != nil {
		return nil, fmt.Errorf("unable to create http request due to error %w", err)
	}

	resp, err := c.cli.Do(req)
	if err != nil {
		switch e := err.(type) {
		case *URL.Error:
			return nil, fmt.Errorf("url.Error received on http request: %w", e)
		default:
			return nil, fmt.Errorf("Unexpected error received: %w", err)
		}
	}

	body, err := FromJSON(resp)
	if err != nil {
		c.logger.Errorf("Client response error: %v", err)
		return nil, err
	}

	return body, nil
}

func (c *Client) get(path string, get, ext bool) ([]byte, error) {
	var url string
	if ext {
		url = fmt.Sprintf("%s%s", c.external, path)
	} else {
		url = fmt.Sprintf("%s%s", c.internal, path)
	}

	method := http.MethodGet
	if !get {
		method = http.MethodDelete
	}

	req, err := http.NewRequest(method, url, bytes.NewBuffer([]byte("")))
	if err != nil {
		return nil, fmt.Errorf("unable to create http request due to error %w", err)
	}

	resp, err := c.cli.Do(req)
	if err != nil {
		switch e := err.(type) {
		case *URL.Error:
			return nil, fmt.Errorf("url.Error received on http request: %w", e)
		default:
			return nil, fmt.Errorf("Unexpected error received: %w", err)
		}
	}

	body, err := FromJSON(resp)
	if err != nil {
		c.logger.Errorf("Client response error: %v", err)
		return nil, err
	}

	return body, nil
}

func (c *Client) PostInt(url string, r interface{}) (body []byte, err error) {
	return c.post(url, r, POST, INTERNAL)
}

func (c *Client) PostExt(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsAliveRemote() {
		return c.post(url, r, POST, EXTERNAL)
	} else {
		return nil, fmt.Errorf("Remote server is dead")
	}
}

func (c *Client) PostMaster(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.PostInt(url, r)
	} else {
		return c.PostExt(url, r)
	}
}

func (c *Client) PostSlave(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.PostExt(url, r)
	} else {
		return c.PostInt(url, r)
	}
}

func (c *Client) PutInt(url string, r interface{}) (body []byte, err error) {
	return c.post(url, r, PUT, INTERNAL)
}

func (c *Client) PutExt(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsAliveRemote() {
		return c.post(url, r, PUT, EXTERNAL)
	} else {
		return nil, fmt.Errorf("Remote server is dead")
	}
}

func (c *Client) PutMaster(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.PutInt(url, r)
	} else {
		return c.PutExt(url, r)
	}
}

func (c *Client) PutSlave(url string, r interface{}) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.PutExt(url, r)
	} else {
		return c.PutInt(url, r)
	}
}

func (c *Client) GetInt(url string) (body []byte, err error) {
	return c.get(url, GET, INTERNAL)
}

func (c *Client) GetExt(url string) (body []byte, err error) {
	if c.lead.IsAliveRemote() {
		return c.get(url, GET, EXTERNAL)
	} else {
		return nil, fmt.Errorf("Remote server is dead")
	}
}

func (c *Client) GetMaster(url string) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.GetInt(url)
	} else {
		return c.GetExt(url)
	}
}

func (c *Client) GetSlave(url string) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.GetExt(url)
	} else {
		return c.GetInt(url)
	}
}

func (c *Client) DeleteInt(url string) (body []byte, err error) {
	return c.get(url, DELETE, INTERNAL)
}

func (c *Client) DeleteExt(url string) (body []byte, err error) {
	if c.lead.IsAliveRemote() {
		return c.get(url, DELETE, EXTERNAL)
	} else {
		return nil, fmt.Errorf("Remote server is dead")
	}
}

func (c *Client) DeleteMaster(url string) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.DeleteInt(url)
	} else {
		return c.DeleteExt(url)
	}
}

func (c *Client) DeleteSlave(url string) (body []byte, err error) {
	if c.lead.IsMaster() {
		return c.DeleteExt(url)
	} else {
		return c.DeleteInt(url)
	}
}
