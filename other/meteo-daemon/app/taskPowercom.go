package app

import (
	"fmt"
	"regexp"
	"time"

	"github.com/sirupsen/logrus"
)

type Powecom struct {
	logger *logrus.Entry
}

func NewPowercom(lg *logrus.Entry) *Powecom {
	return &Powecom{
		logger: lg,
	}
}

func (p *Powecom) Run(params map[string]string) (err error) {
	if len(params) != 0 {
		return fmt.Errorf("Invalid number of parameters: %d, required 0", len(params))
	}
	timeout := time.After(3 * time.Second)
	ch := make(chan []byte, 10)

	go execCmd("upsc powercom", ch, p.logger)

	select {
	case out := <-ch:
		matched, _ := regexp.MatchString("ups.status: OL", string(out))
		if !matched {
			go execCmd("upsdrvctl shutdown", ch, p.logger)
			select {
			case out := <-ch:
				matched, _ := regexp.MatchString("Initiating UPS shutdown", string(out))
				if !matched {
					p.logger.Error("Powecom shutdown error")
				} else {
					time.Sleep(2 * time.Second)
					go execCmd("upsdrvctl start powercom", ch, p.logger)
					select {
					case out := <-ch:
						p.logger.Infof("Powercom start: %v", string(out))
					case <-timeout:
						p.logger.Error(msgTimedOutCommand)
					}
				}
			case <-timeout:
				p.logger.Error(msgTimedOutCommand)
			}
		}
	case <-timeout:
		p.logger.Error(msgTimedOutCommand)
	}
	return nil
}

func (p *Powecom) Terminate() error {
	return nil
}
