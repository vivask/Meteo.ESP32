package web

import (
	"context"
	"time"
)

func (web *Web) firmwareWathdog() {
	counter := 0

	for {
		counter++
		if row, err := web.repo.GetSettings(context.Background()); err != nil {
			web.logger.Error(err)
		} else {
			if row.UpgradeStatus != 0 {
				web.logger.Debug("Watchdog firmware upgrade success")
				return
			}
		}
		if counter > WATCHDOG_FIRMWARE_RETRY {
			err := web.repo.TerminateUpgrade(context.Background())
			if err != nil {
				web.logger.Errorf("Triminating upgrade fail: %v", err)
				return
			}
			web.logger.Info("Watchdog firmware upgrade terminate")
			return
		}
		time.Sleep(WATCHDOG_FIRMWARE_TIMER * time.Second)
	}
}
