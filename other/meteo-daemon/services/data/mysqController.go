package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) StartUpgradeEsp32(ctx context.Context, fName string) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.Firmware = fName
	set.UpgradeStatus = 0
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success StartUpgradeEsp32")
	return nil
}

func (r *MysqlRepository) GetUpgradeStatus(ctx context.Context) (*Settings, error) {
	return r.GetSettings(ctx)
}

func (r *MysqlRepository) StartAccesPointMode(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.SetupMode = true
	set.SetupStatus = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success StartAccesPointMode")
	return nil
}

func (r *MysqlRepository) GetStatusAccesPoint(ctx context.Context) (*Settings, error) {
	return r.GetSettings(ctx)
}

func (r *MysqlRepository) StartRebootEsp32(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.Reboot = true
	set.Rebooted = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success StartRebootEsp32")
	return nil
}

func (r *MysqlRepository) GetRebootStatus(ctx context.Context) (*Settings, error) {
	return r.GetSettings(ctx)
}
