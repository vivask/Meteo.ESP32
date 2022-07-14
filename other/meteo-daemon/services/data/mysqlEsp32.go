package data

import (
	"context"
	"fmt"
	hs "meteo-daemon/services/data/internal"
	"strconv"
	"sync"
	"time"
)

var mux sync.Mutex

var (
	lockBmx280   = false
	lockDs18b20  = false
	lockZe08ch2o = false
	lockRadsens  = false
)

func (r *MysqlRepository) UnlockBmx280(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/unlock/bmx280", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockBmx280 = false
	mux.Unlock()
	return nil
}

func (r *MysqlRepository) LockBmx280(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/lock/bmx280", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockBmx280 = true
	mux.Unlock()
	return nil
}

func isLockedBmx280() bool {
	mux.Lock()
	defer mux.Unlock()
	return lockBmx280
}

func (r *MysqlRepository) UnlockDs18b20(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/unlock/ds18b20", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockDs18b20 = false
	mux.Unlock()
	return nil
}

func (r *MysqlRepository) LockDs18b20(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/lock/ds18b20", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockDs18b20 = true
	mux.Unlock()
	return nil
}

func isLockedDs18b20() bool {
	mux.Lock()
	defer mux.Unlock()
	return lockDs18b20
}

func (r *MysqlRepository) UnlockZe08ch2o(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/unlock/ze08ch2o", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockZe08ch2o = false
	mux.Unlock()
	return nil
}

func (r *MysqlRepository) LockZe08ch2o(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/lock/ze08ch2o", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockZe08ch2o = true
	mux.Unlock()
	return nil
}

func isLockedZe08ch2o() bool {
	mux.Lock()
	defer mux.Unlock()
	return lockZe08ch2o
}

func (r *MysqlRepository) UnlockRadsens(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/unlock/radsens", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockRadsens = false
	mux.Unlock()
	return nil
}

func (r *MysqlRepository) LockRadsens(ext bool) error {
	if ext {
		_, err := r.cli.PutExt("/verify/db/rest/lock/radsens", nil)
		if err != nil {
			return fmt.Errorf("error PUT: %w", err)
		}
	}
	mux.Lock()
	lockRadsens = true
	mux.Unlock()
	return nil
}

func isLockedRadsens() bool {
	mux.Lock()
	defer mux.Unlock()
	return lockRadsens
}

func toFloat(src interface{}) (float64, error) {
	fs, ok := src.(string)
	if !ok {
		return 0, fmt.Errorf("convert interface to string: %v", src)
	}
	return strconv.ParseFloat(fs, 64)
}

func toInt(src interface{}) (int, error) {
	is, ok := src.(string)
	if !ok {
		return 0, fmt.Errorf("convert interface to string: %v", src)
	}
	return strconv.Atoi(is)
}

func toTime(src interface{}) (time.Time, error) {
	ts, ok := src.(string)
	if !ok {
		return time.Time{}, fmt.Errorf("convert interface to string: %v", src)
	}
	return time.ParseInLocation("2006-01-02 15:04:05", ts, time.Local)
}

func toBool(src interface{}) (bool, error) {
	i, err := toInt(src)
	if err != nil {
		return false, err
	}
	ret := false
	if i != 0 {
		ret = true
	}
	return ret, nil
}

func (r *MysqlRepository) TerminateUpgrade(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.Firmware = "_EMPTY_"
	set.UpgradeStatus = -1
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debug("sucess terminate upgrade")
	return nil
}

func (r *MysqlRepository) AddLoging(ctx context.Context, msg, t, dts interface{}) error {
	message, ok := msg.(string)
	if !ok {
		return fmt.Errorf("convert interface to string: %v", msg)
	}
	msgType, ok := t.(string)
	if !ok {
		return fmt.Errorf("convert interface to string: %v", t)
	}
	dtstring, ok := dts.(string)
	if !ok {
		return fmt.Errorf("convert interface to string: %v", dts)
	}
	dt, err := time.ParseInLocation("2006-01-02 15:04:05", dtstring, time.Local)
	if err != nil {
		return fmt.Errorf("parse time error: %w", err)
	}
	logging := Logging{ID: hs.HashTime32(dt), Message: message, Type: msgType, CreatedAt: dt}
	err = r.db.Create(&logging).Error
	if err != nil {
		return fmt.Errorf("error insert logging: %w", err)
	}
	r.logger.Debugf("crete logging: %v", logging)
	return nil
}

func (r *MysqlRepository) AddDs18b20(ctx context.Context, tempr interface{}) error {
	if isLockedDs18b20() {
		return nil
	}

	temperature, err := toFloat(tempr)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	ds18b20 := Ds18b20{ID: hs.HashNow32(), Tempr: temperature}
	err = r.db.Create(&ds18b20).Error
	if err != nil {
		return fmt.Errorf("insert ds18b20: %v, error: %w", ds18b20, err)
	}
	r.logger.Infof("create ds18b20: %v", ds18b20)
	settings, err := r.GetSettings(ctx)
	if err != nil {
		return fmt.Errorf("error read settings: %w", err)
	}
	message := fmt.Sprintf("DS18B20: Температура воздуха: %f°C", temperature)
	if temperature < settings.MinDs18b20 {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MinDs18b20Alarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	if temperature > settings.MaxDs18b20 && !settings.MaxDs18b20Alarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MaxDs18b20Alarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	return err
}

func (r *MysqlRepository) AddBmx280(ctx context.Context, press, tempr, hum interface{}) error {
	if isLockedBmx280() {
		return nil
	}

	temperature, err := toFloat(tempr)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	pressure, err := toFloat(press)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	humidity, err := toFloat(hum)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	bmx280 := Bmx280{ID: hs.HashNow32(), Press: pressure, Tempr: temperature, Hum: humidity}
	err = r.db.Create(&bmx280).Error
	if err != nil {
		return fmt.Errorf("insert bmx280: %v, error: %w", bmx280, err)
	}
	r.logger.Infof("cretae bmx280: %v", bmx280)
	settings, err := r.GetSettings(ctx)
	if err != nil {
		return fmt.Errorf("error read settings: %w", err)
	}
	message := fmt.Sprintf("BMX280: Температура воздуха: %f°C", temperature)
	if temperature < settings.MinBmx280Tempr && !settings.MinBmx280TemprAlarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MinBmx280TemprAlarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	if temperature > settings.MaxBmx280Tempr && !settings.MinBmx280TemprAlarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MinBmx280TemprAlarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	return err
}

func (r *MysqlRepository) AddRadsens(ctx context.Context, dyn, stat, pl interface{}) error {
	if isLockedRadsens() {
		return nil
	}

	static, err := toFloat(stat)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	dynamic, err := toFloat(dyn)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	pulse, err := toInt(pl)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	radsens := Radsens{ID: hs.HashNow32(), Dynamic: dynamic, Static: static, Pulse: pulse}
	err = r.db.Create(&radsens).Error
	if err != nil {
		return fmt.Errorf("insert radsens: %v, error: %w", radsens, err)
	}
	r.logger.Infof("create radsens: %v", radsens)
	settings, err := r.GetSettings(ctx)
	if err != nil {
		return fmt.Errorf("error read settings: %w", err)
	}
	message := fmt.Sprintf("RadSens: Превышена динамическая интенсивность излучения: %f мкР/ч", dynamic)
	if dynamic > settings.MaxRadDyn && !settings.MaxRadDynAlarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MaxRadDynAlarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	message = fmt.Sprintf("RadSens: Превышена статическая интенсивность излучения: %f мкР/ч", static)
	if static > settings.MaxRadStat && !settings.MaxRadStatAlarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MaxRadStatAlarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	return err
}

func (r *MysqlRepository) AddZe08ch2o(ctx context.Context, ch2o interface{}) error {
	if isLockedZe08ch2o() {
		return nil
	}

	value, err := toInt(ch2o)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	ze08ch2o := Ze08ch2o{ID: hs.HashNow32(), Ch2o: value}
	err = r.db.Create(&ze08ch2o).Error
	if err != nil {
		return fmt.Errorf("insert ze08ch2o: %v, error: %w", ze08ch2o, err)
	}
	r.logger.Infof("create ze08ch2o: %v", ze08ch2o)
	settings, err := r.GetSettings(ctx)
	if err != nil {
		return fmt.Errorf("error read settings: %w", err)
	}
	message := fmt.Sprintf("ZE08CH2O: Превышена концентрация CH2O: %d ppm", value)
	if value > settings.MaxCh2o && !settings.MaxCh2oAlarm {
		_, err := r.cli.PostInt("/verify/telegram/rest/message", message)
		if err != nil {
			r.logger.Errorf("can't send telegram message: %v", err)
		}
		err = r.db.Model(&settings).Updates(Settings{MaxCh2oAlarm: true}).Error
		if err != nil {
			return fmt.Errorf("update settings error: %w", err)
		}
	}
	return nil
}

func (r *MysqlRepository) GetEsp32Settings(ctx context.Context, dti interface{}) (*Settings, error) {
	dt, err := toTime(dti)
	if err != nil {
		return nil, fmt.Errorf("convert error: %w", err)
	}
	var settings Settings
	err = r.db.Model(&settings).Updates(Settings{Esp32DateTimeNow: dt}).Error
	if err != nil {
		return nil, fmt.Errorf("update settings error: %w", err)
	}
	err = r.db.First(&settings).Error
	if err != nil {
		return nil, fmt.Errorf("error read settings: %w", err)
	}
	return &settings, nil
}

func (r *MysqlRepository) SetHVRadsens(ctx context.Context, state interface{}) error {
	need_state, err := toBool(state)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	set := Settings{}
	err = r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.RadsensHVMode = false
	set.RadsensHVState = need_state
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("success SetRadsensHV: %v", need_state)
	return nil
}

func (r *MysqlRepository) SetSensRadsens(ctx context.Context, sens interface{}) error {
	s, err := toInt(sens)
	if err != nil {
		return fmt.Errorf("convert error: %w", err)
	}
	set := Settings{}
	err = r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.RadsensSensitivity = s
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("save radsens_sensitivity: %v", s)
	return nil
}

func (r *MysqlRepository) SetAccesPointMode(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.SetupMode = false
	set.SetupStatus = true
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("save setup_mode: %v", false)
	return nil
}

func (r *MysqlRepository) Esp32Reboot(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.Reboot = false
	set.Rebooted = true
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("save rebooted: %v", true)
	return nil
}

func (r *MysqlRepository) SuccessUpgrade(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.Firmware = "_EMPTY_"
	set.UpgradeStatus = 1
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("save UpgradeStatus: %v", true)
	return nil
}

func (r *MysqlRepository) JournaCleared(ctx context.Context) error {
	set := Settings{}
	err := r.db.First(&set).Error
	if err != nil {
		return fmt.Errorf("read settings error: %w", err)
	}
	set.ClearJournalEsp32 = false
	err = r.db.Save(&set).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	r.logger.Debugf("save ClearJournalEsp32: %v", false)
	return nil
}
