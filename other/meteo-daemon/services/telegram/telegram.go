package telegram

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"meteo-daemon/domain"
	"meteo-daemon/utils"
	"net/http"
	"os"

	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp   domain.StartStopInterface
	Logger *logrus.Entry
}

type Telegram struct {
	hasp    domain.StartStopInterface
	logger  *logrus.Entry
	config  *Config
	logfile *os.File
}

func New(cnf *Config, p *Params) *Telegram {

	t := &Telegram{
		hasp:   p.Hasp,
		logger: p.Logger,
		config: cnf,
	}

	return t
}

func (t *Telegram) Message(msg string) error {
	type sendMessageReqBody struct {
		ChatID int64  `json:"chat_id"`
		Text   string `json:"text"`
	}

	reqBody := &sendMessageReqBody{
		ChatID: t.config.ChatId,
		Text:   msg,
	}

	reqBytes, err := json.Marshal(reqBody)
	if err != nil {
		t.logger.Error(err)
		return err
	}

	url := t.config.Url + t.config.Key + "/sendMessage"
	res, err := http.Post(url, "application/json", bytes.NewBuffer(reqBytes))
	if err != nil {
		t.logger.Error(err)
		return err
	}
	if res.StatusCode != http.StatusOK {
		err = errors.New("Unexpected status" + res.Status)
		t.logger.Error(err)
		return err
	}

	return nil
}

func (Telegram) StartNum() int {
	return 4
}

func (t *Telegram) Enabled() bool {
	return t.config.Active
}

func (t *Telegram) IsRun() bool {
	return t.hasp.IsRun()
}

func (t *Telegram) Start() error {
	if !t.hasp.Start() {
		return fmt.Errorf("%s:failed to start", t.config.Title)
	}

	t.logger.Debugf("%s: success started", t.config.Title)

	return nil
}

func (t *Telegram) Stop() error {
	if !t.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", t.config.Title)
	}

	t.logger.Debugf("%s: success stoped", t.config.Title)

	if t.logfile != nil {
		t.logfile.Close()
	}

	return nil
}

func (t *Telegram) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, t.config.LogFile, t.config.LogLevel, file, t.logger)
	if err != nil {
		return err
	}
	t.logfile = f
	return nil
}

func (t *Telegram) Terminate() error {
	return nil
}

//params[0].([]interface{})[0].(string)
func (t *Telegram) Run(params map[string]string) error {
	var err error
	if len(params) != 1 {
		err = fmt.Errorf("Invalid number of parameters: %d, required 1", len(params))
	} else {
		if value, found := params["msg"]; found {
			err = t.Message(value)
		} else {
			err = fmt.Errorf("Parameter 'msg' not found")
		}
	}

	return err
}

type Config struct {
	Title    string `toml:"Title"`
	Active   bool   `toml:"Active"`
	Url      string `toml:"Url"`
	Key      string `toml:"Key"`
	ChatId   int64  `toml:"ChatId"`
	LogFile  string `toml:"LogFile"`
	LogLevel string `toml:"LogLevel"`
}
