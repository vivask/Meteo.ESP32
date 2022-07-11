package app

import (
	"errors"
	"fmt"
	"meteo-daemon/client"
	"os"
	"os/exec"
	"time"

	"github.com/sirupsen/logrus"
)

const (
	msgStorageNotAvailable = "Хранилище недоступно"
	msgStorageAvailable    = "Доступ к хранилищу восстановлен"
	msgTimedOutCommand     = "Время выполнения команды истекло"
)

var (
	STORAGE_MOUNTED = true
)

type StorageMount struct {
	cli          *client.Client
	logger       *logrus.Entry
	mountPoint   string
	mountUnit    string
	timeoutMount int
}

func (s *StorageMount) Run(params map[string]string) (err error) {
	if len(params) != 0 {
		return fmt.Errorf("Invalid number of parameters: %d, required 0", len(params))
	}

	if _, err := os.Stat(s.mountPoint); errors.Is(err, os.ErrNotExist) {
		if STORAGE_MOUNTED {
			timeout := time.After(time.Duration(s.timeoutMount) * time.Second)
			ch := make(chan []byte, 10)

			STORAGE_MOUNTED = false
			s.logger.Info(msgStorageNotAvailable)
			_, err := s.cli.PostInt("/verify/telegram/rest/message", msgStorageNotAvailable)
			if err != nil {
				s.logger.Errorf("can't send telegram message: %v", err)
			}

			go execCmd("systemctl start "+s.mountUnit, ch, s.logger)
			select {
			case <-ch:
				if _, err := os.Stat(s.mountPoint); err == nil {
					STORAGE_MOUNTED = true
					s.logger.Info(msgStorageAvailable)
					_, err := s.cli.PostInt("/verify/telegram/rest/message", msgStorageNotAvailable)
					if err != nil {
						s.logger.Errorf("can't send telegram message: %v", err)
					}
				}
			case <-timeout:
				s.logger.Error(msgTimedOutCommand)
			}
		}
	}

	return nil
}

func (s *StorageMount) Terminate() error {
	return nil
}

func execCmd(cmd string, ch chan []byte, logger *logrus.Entry) {
	out, err := exec.Command("sh", "-c", cmd).Output()
	if err != nil {
		logger.Error(err)
	}
	ch <- out
}
