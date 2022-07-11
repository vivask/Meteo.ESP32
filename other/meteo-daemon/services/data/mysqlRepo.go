package data

import (
	"context"
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/utils"
	"os"
	"strings"
	s "strings"
	"time"

	"github.com/jinzhu/gorm"
	"github.com/sirupsen/logrus"
)

type MysqlRepository struct {
	hasp       domain.StartStopInterface
	logger     *logrus.Entry
	config     *Config
	db         *gorm.DB
	cli        *client.Client
	logfile    *os.File
	tables     map[int]string
	ignoreSync map[string]struct{}
	lead       *leader.Leader
	sync       bool
}

func NewMysqlRepository(cnf *Config, p *Params) *MysqlRepository {
	m := map[string]struct{}{}
	for _, ignore := range cnf.IgnoreSync {
		m[ignore] = struct{}{}
	}

	return &MysqlRepository{
		hasp:       p.Hasp,
		logger:     p.Logger,
		config:     cnf,
		db:         p.Conn,
		cli:        p.Cli,
		ignoreSync: m,
		lead:       p.Lead,
		sync:       cnf.Sync,
	}
}

func (r *MysqlRepository) IsSync() bool {
	return r.sync
}

func (r *MysqlRepository) OnSyncAuto() {
	r.sync = true
}

func (r *MysqlRepository) OffSyncAuto() {
	r.sync = false
}

func (r *MysqlRepository) IgnoreAutoSyncTable(table string) {
	r.ignoreSync[table] = struct{}{}
}

func (r *MysqlRepository) RestoreAutoSyncTable(table string) {
	delete(r.ignoreSync, table)
}

func (r *MysqlRepository) Initialize(ctx context.Context) (err error) {
	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Bmx280{}).Error
	if err != nil {
		return fmt.Errorf("create table bmx280 error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Mics6814{}).Error
	if err != nil {
		return fmt.Errorf("create table mics6814 error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Ds18b20{}).Error
	if err != nil {
		return fmt.Errorf("create table ds18b20 error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Radsens{}).Error
	if err != nil {
		return fmt.Errorf("create table radsens error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Ze08ch2o{}).Error
	if err != nil {
		return fmt.Errorf("create table ze08ch2o error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Logging{}).Error
	if err != nil {
		return fmt.Errorf("create table logging error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Settings{}).Error
	if err != nil {
		return fmt.Errorf("create table settings error: %w", err)
	}
	if r.db.First(&Settings{}).Error == gorm.ErrRecordNotFound {
		err := r.db.Create(&Settings{ID: 1}).Error
		if err != nil {
			return fmt.Errorf("insert settings error: %w", err)
		}
		r.logger.Debugf("save settings: initialize")
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&TaskParams{}).Error
	if err != nil {
		return fmt.Errorf("create table task_params error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Tasks{}).Error
	if err != nil {
		return fmt.Errorf("create table tasks error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Executors{}).Error
	if err != nil {
		return fmt.Errorf("create table executors error: %w", err)
	}
	if r.db.First(&Executors{}).Error == gorm.ErrRecordNotFound {
		executors := []Executors{
			{ID: "Master"},
			{ID: "Leader"},
			{ID: "Slave"},
			{ID: "All"},
		}
		for _, executor := range executors {
			err := r.db.Create(&executor).Error
			if err != nil {
				return fmt.Errorf("insert executors error: %w", err)
			}
		}
		r.logger.Debugf("save executors: initialize")
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Periods{}).Error
	if err != nil {
		return fmt.Errorf("create table periods error: %w", err)
	}
	if r.db.First(&Periods{}).Error == gorm.ErrRecordNotFound {
		periods := []Periods{
			{ID: "one", Name: "Единоразово", Idx: 1},
			{ID: "sec", Name: "Секунда", Idx: 2},
			{ID: "min", Name: "Минута", Idx: 3},
			{ID: "hour", Name: "Час", Idx: 4},
			{ID: "day", Name: "День", Idx: 5},
			{ID: "week", Name: "Неделя", Idx: 6},
			{ID: "day_of_week", Name: "День недели", Idx: 7},
			{ID: "month", Name: "Месяц", Idx: 8},
		}
		for _, period := range periods {
			err := r.db.Create(&period).Error
			if err != nil {
				return fmt.Errorf("insert periods error: %w", err)
			}
		}
		r.logger.Debugf("save periods: initialize")
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Days{}).Error
	if err != nil {
		return fmt.Errorf("create table days error: %w", err)
	}
	if r.db.First(&Days{}).Error == gorm.ErrRecordNotFound {
		days := []Days{
			{ID: 1, Name: "Понедельник"},
			{ID: 2, Name: "Вторник"},
			{ID: 3, Name: "Среду"},
			{ID: 4, Name: "Четверг"},
			{ID: 5, Name: "Пятницу"},
			{ID: 6, Name: "Субботу"},
			{ID: 7, Name: "Воскресенье"},
		}
		for _, day := range days {
			err := r.db.Create(&day).Error
			if err != nil {
				return fmt.Errorf("insert days error: %w", err)
			}
		}
		r.logger.Debugf("save days: initialize")
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&JobParams{}).Error
	if err != nil {
		return fmt.Errorf("create table job_params error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Jobs{}).Error
	if err != nil {
		return fmt.Errorf("create table jobs error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&SSHKeys{}).Error
	if err != nil {
		return fmt.Errorf("create table ssh_keys error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Knowhosts{}).Error
	if err != nil {
		return fmt.Errorf("create table knowhosts error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&GitKeys{}).Error
	if err != nil {
		return fmt.Errorf("create table git_keys error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&GitUsers{}).Error
	if err != nil {
		return fmt.Errorf("create table git_users error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Blocklist{}).Error
	if err != nil {
		return fmt.Errorf("create table blocklists error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&Homezone{}).Error
	if err != nil {
		return fmt.Errorf("create table homezones error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&AccesList{}).Error
	if err != nil {
		return fmt.Errorf("create table access_lists error: %w", err)
	}
	if r.db.First(&AccesList{}).Error == gorm.ErrRecordNotFound {
		lists := []AccesList{
			{ID: "tovpn"},
			{ID: "local"},
		}
		for _, list := range lists {
			err := r.db.Create(&list).Error
			if err != nil {
				return fmt.Errorf("insert access_lists error: %w", err)
			}
		}
		r.logger.Debugf("save access_lists: initialize")
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&ToVpnManual{}).Error
	if err != nil {
		return fmt.Errorf("create table to_vpn_manuals error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&ToVpnAuto{}).Error
	if err != nil {
		return fmt.Errorf("create table to_vpn_autos error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&ToVpnIgnore{}).Error
	if err != nil {
		return fmt.Errorf("create table to_vpn_ignores error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&User{}).Error
	if err != nil {
		return fmt.Errorf("create table users error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&SyncTables{}).Error
	if err != nil {
		return fmt.Errorf("create table sync_tables error: %w", err)
	}

	err = r.db.Set("gorm:table_options", "ENGINE=InnoDB").AutoMigrate(&SyncParams{}).Error
	if err != nil {
		return fmt.Errorf("create table sync_params error: %w", err)
	}

	r.db.Callback().Create().After("gorm:after_create").Register("sync:sync_create", r.syncCreate)
	r.db.Callback().Update().After("gorm:after_update").Register("sync:sync_update", r.syncUpdate)
	r.db.Callback().Delete().After("gorm:after_delete").Register("sync:sync_delete", r.syncDelete)

	return nil
}

func params(params []interface{}) []interface{} {
	prep := []interface{}{}
	for _, param := range params {
		if dt, ok := param.(time.Time); ok {
			dts := dt.Format("2006-01-02 15:04:05")
			prep = append(prep, dts)
		} else {
			prep = append(prep, param)
		}
	}
	return prep
}

func ParseQuery(query string) (action, name string, err error) {
	name = "unknow"
	words := s.Split(query, " ")
	action = words[0]
	switch action {
	case "UPDATE":
		name = words[1]
	case "INSERT":
		name = words[2]
	case "DELETE":
		name = words[2]
	default:
		err = fmt.Errorf("Unknown SQL query: %s", words[0])
		return
	}
	return action, strings.Replace(name, "`", "", -1), nil
}

func (r *MysqlRepository) syncCreate(scope *gorm.Scope) {
	if r.IsSync() {
		r.logger.Debug(scope.SQL)
		r.logger.Debug(scope.SQLVars)
		_, tblName, err := ParseQuery(scope.SQL)
		if err != nil {
			r.logger.Error(err)
			return
		}
		if _, ok := r.ignoreSync[tblName]; ok {
			return
		}
		_, err = r.cli.PostExt("/verify/db/rest/exec", Callback{Query: scope.SQL, Params: params(scope.SQLVars)})
		if err != nil {
			r.logger.Error(err)
		}
	}
}

func (r *MysqlRepository) syncUpdate(scope *gorm.Scope) {
	if r.IsSync() {
		r.logger.Debug(scope.SQL)
		r.logger.Debug(scope.SQLVars)
		_, tblName, err := ParseQuery(scope.SQL)
		if err != nil {
			r.logger.Error(err)
			return
		}
		if _, ok := r.ignoreSync[tblName]; ok {
			return
		}
		_, err = r.cli.PostExt("/verify/db/rest/exec", Callback{Query: scope.SQL, Params: params(scope.SQLVars)})
		if err != nil {
			r.logger.Error(err)
		}
	}
}

func (r *MysqlRepository) syncDelete(scope *gorm.Scope) {
	if r.IsSync() {
		r.logger.Debug(scope.SQL)
		r.logger.Debug(scope.SQLVars)
		_, tblName, err := ParseQuery(scope.SQL)
		if err != nil {
			r.logger.Error(err)
			return
		}
		if _, ok := r.ignoreSync[tblName]; ok {
			return
		}
		_, err = r.cli.PostExt("/verify/db/rest/exec", Callback{Query: scope.SQL, Params: params(scope.SQLVars)})
		if err != nil {
			r.logger.Error(err)
		}
	}
}

func (r *MysqlRepository) ExecRaw(ctx context.Context, cb Callback) error {
	err := r.db.Exec(cb.Query, cb.Params...).Error
	if err != nil {
		r.logger.Debug(cb.Query)
		r.logger.Debug(cb.Params...)
		return fmt.Errorf("exec error: %w", err)
	}
	return nil
}

func (MysqlRepository) StartNum() int {
	return 1
}

func (MysqlRepository) Enabled() bool {
	return true
}

func (r *MysqlRepository) IsRun() bool {
	return r.hasp.IsRun()
}

func (r *MysqlRepository) Start() error {
	if !r.hasp.Start() {
		return fmt.Errorf("%s:failed to start", r.config.Title)
	}

	r.logger.Debugf("%s: success started", r.config.Title)

	return nil
}

func (r *MysqlRepository) Stop() error {
	if !r.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", r.config.Title)
	}

	r.db.Close()

	r.logger.Debugf("%s: success stoped", r.config.Title)
	if r.logfile != nil {
		r.logfile.Close()
	}

	return nil
}

func (r *MysqlRepository) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, r.config.LogFile, r.config.LogLevel, file, r.logger)
	if err != nil {
		return err
	}
	r.logfile = f
	return nil
}
