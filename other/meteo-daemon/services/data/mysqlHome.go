package data

import (
	"context"
	"fmt"

	"github.com/jinzhu/gorm"
)

func (r *MysqlRepository) GetDs18b20(ctx context.Context) (*Ds18b20, error) {
	var ds18b20 Ds18b20
	err := r.db.Order("date_time desc").Last(&ds18b20).Error
	if err != nil {
		if err == gorm.ErrRecordNotFound {
			return &Ds18b20{}, nil
		}
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return &ds18b20, err
}

func (r *MysqlRepository) GetBmx280(ctx context.Context) (*Bmx280, error) {
	var bmx280 Bmx280
	err := r.db.Order("date_time desc").Last(&bmx280).Error
	if err != nil {
		if err == gorm.ErrRecordNotFound {
			return &Bmx280{}, nil
		}
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read Bmx280: %v", bmx280)
	return &bmx280, err
}

func (r *MysqlRepository) GetMics6814(ctx context.Context) (*Mics6814, error) {
	var mics6814 Mics6814
	err := r.db.Order("date_time desc").Last(&mics6814).Error
	if err != nil {
		if err == gorm.ErrRecordNotFound {
			return &Mics6814{}, nil
		}
		return nil, fmt.Errorf("error read mics6814: %w", err)
	}
	r.logger.Debugf("read mics6814: %v", mics6814)
	return &mics6814, err
}

func (r *MysqlRepository) GetRadsens(ctx context.Context) (*Radsens, error) {
	var radsens Radsens
	err := r.db.Order("date_time desc").Last(&radsens).Error
	if err != nil {
		if err == gorm.ErrRecordNotFound {
			return &Radsens{}, nil
		}
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return &radsens, err
}

func (r *MysqlRepository) GetZe08ch2o(ctx context.Context) (*Ze08ch2o, error) {
	var ze08ch2o Ze08ch2o
	err := r.db.Order("date_time desc").Last(&ze08ch2o).Error
	if err != nil {
		if err == gorm.ErrRecordNotFound {
			return &Ze08ch2o{}, nil
		}
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return &ze08ch2o, err
}

func (r *MysqlRepository) GetEsp32DateTime(ctx context.Context) (*Esp32DateTime, error) {
	var result Esp32DateTime
	query := `SELECT DATE_FORMAT(esp32_date_time_now,'%d/%m/%Y %H:%i') AS esp32_date_time,
			  esp32_date_time_now, updatedat
			  FROM settings LIMIT 1`
	err := r.db.Raw(query).Scan(&result).Error
	if err != nil {
		return nil, fmt.Errorf("error read Esp32DateTime: %w", err)
	}
	//r.logger.Debugf("read Esp32DateTime: %v", result)
	return &result, err
}

func (r *MysqlRepository) CheckBmx280Tempr(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.MaxBmx280TemprAlarm = false
	set.MinBmx280TemprAlarm = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success CheckBmx280Tempr")
	return nil
}

func (r *MysqlRepository) CheckDs18b20(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.MaxDs18b20Alarm = false
	set.MaxDs18b20Alarm = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success CheckDs18b20")
	return nil
}

func (r *MysqlRepository) CheckZe08ch2o(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.MaxCh2oAlarm = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success CheckZe08ch2o")
	return nil
}

func (r *MysqlRepository) CheckRadsensDyn(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.MaxRadDynAlarm = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success CheckRadsensDyn")
	return nil
}

func (r *MysqlRepository) CheckRadsensStat(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.MaxRadStatAlarm = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success CheckRadsensStat")
	return nil
}

func (r *MysqlRepository) SetRadsensHV(ctx context.Context, state bool) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.RadsensHVMode = true
	set.RadsensHVState = state
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success SetRadsensHV: %v", state)
	return nil
}
