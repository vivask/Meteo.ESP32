package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetSettings(ctx context.Context) (*Settings, error) {
	var settings Settings
	err := r.db.First(&settings).Error
	if err != nil {
		return nil, fmt.Errorf("error read settings: %w", err)
	}
	r.logger.Debugf("read Settings: %v", settings)
	return &settings, err
}

func (r *MysqlRepository) SetSettings(ctx context.Context, s *Settings) error {
	err := r.db.Save(s).Error
	if err != nil {
		return fmt.Errorf("error save settings: %w", err)
	}
	r.logger.Debugf("save Settings: %v", s)
	return nil
}
