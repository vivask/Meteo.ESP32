package data

import (
	"time"
)

type Bmx280 struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Press     float64   `gorm:"column:press;not null" json:"press"`
	Tempr     float64   `gorm:"column:tempr;not null" json:"tempr"`
	Hum       float64   `gorm:"column:hum;not null" json:"hum"`
	CreatedAt time.Time `gorm:"column:date_time;not null;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Bmx280) TableName() string {
	return "bmx280"
}

type Mics6814 struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	No2       float64   `gorm:"column:no2;not null" json:"no2"`
	Nh3       float64   `gorm:"column:nh3;not null" json:"nh3"`
	Co        float64   `gorm:"column:co;not null" json:"co"`
	CreatedAt time.Time `gorm:"column:date_time;not null;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Mics6814) TableName() string {
	return "mics6814"
}

type Ds18b20 struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Tempr     float64   `gorm:"column:tempr;not null" json:"tempr"`
	CreatedAt time.Time `gorm:"column:date_time;not null;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Ds18b20) TableName() string {
	return "ds18b20"
}

type Radsens struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Dynamic   float64   `gorm:"column:dynamic;not null" json:"dynamic"`
	Static    float64   `gorm:"column:static;not null" json:"static"`
	Pulse     int       `gorm:"column:pulse;not null" json:"pulse"`
	CreatedAt time.Time `gorm:"column:date_time;not null;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Radsens) TableName() string {
	return "radsens"
}

type Ze08ch2o struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Ch2o      int       `gorm:"column:ch2o;not null" json:"ch2o"`
	CreatedAt time.Time `gorm:"column:date_time;not null;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Ze08ch2o) TableName() string {
	return "ze08ch2o"
}

type Logging struct {
	ID        uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Message   string    `gorm:"column:message;not null;size:128" json:"message"`
	Type      string    `gorm:"column:type;not null;size:2" json:"type"`
	CreatedAt time.Time `gorm:"column:date_time;not null;autoCreateTime:false;default:CURRENT_TIMESTAMP" json:"date_time"`
	Date      string    `gorm:"-" sql:"date" json:"date"`
	Time      string    `gorm:"-" sql:"time" json:"time"`
}

func (Logging) TableName() string {
	return "logging"
}

type Settings struct {
	ID                    uint32    `gorm:"column:id;not null;primaryKey;unique" json:"id"`
	ValveState            bool      `gorm:"column:valve_state;not null;default:false" json:"valve_state"`
	ValveDisable          bool      `gorm:"column:valve_disable;not null;default:false" json:"valve_disable"`
	MinTempn              float64   `gorm:"column:min_temp;not null;default:9.0" json:"min_temp"`
	MaxTemp               float64   `gorm:"column:max_temp;not null;default:12.0" json:"max_temp"`
	CCS811Baseline        int       `gorm:"column:ccs811_baseline;not null;default:0" json:"CCS811_baseline"`
	Firmware              string    `gorm:"column:firmware;not null;default:'_EMPTY_'" json:"firmware"`
	UpgradeStatus         int       `gorm:"column:upgrade_status;not null;default:1" json:"upgrade_status"`
	SetupMode             bool      `gorm:"column:setup_mode;not null;default:false" json:"setup_mode"`
	SetupStatus           bool      `gorm:"column:setup_status;not null;default:true" json:"setup_status"`
	Reboot                bool      `gorm:"column:reboot;not null;default:false" json:"reboot"`
	Rebooted              bool      `gorm:"column:rebooted;not null;default:true" json:"rebooted"`
	MaxCh2o               int       `gorm:"column:max_ch2o;not null;default:150" json:"max_ch2o"`
	MaxCh2oAlarm          bool      `gorm:"column:max_ch2o_alarm;not null;default:false" json:"max_ch2o_alarm"`
	MaxDs18b20            float64   `gorm:"column:max_ds18b20;not null;default:30.0" json:"max_ds18b20"`
	MinDs18b20            float64   `gorm:"column:min_ds18b20;not null;default:8.0" json:"min_ds18b20"`
	MaxDs18b20Alarm       bool      `gorm:"column:max_ds18b20_alarm;not null;default:false" json:"max_ds18b20_alarm"`
	MinDs18b20Alarm       bool      `gorm:"column:min_ds18b20_alarm;not null;default:false" json:"min_ds18b20_alarm"`
	Max6814Nh3            float64   `gorm:"column:max_6814_nh3;not null;default:0.0" json:"max_6814_nh3"`
	Max6814Co             float64   `gorm:"column:max_6814_co;not null;default:0.0" json:"max_6814_co"`
	Max6814No2            float64   `gorm:"column:max_6814_no2;not null;default:0.0" json:"max_6814_no2"`
	Max6814Nh3Alarm       bool      `gorm:"column:max_6814_nh3_alarm;not null;default:false" json:"max_6814_nh3_alarm"`
	Max6814CoAlarm        bool      `gorm:"column:max_6814_co_alarm;not null;default:false" json:"max_6814_co_alarm"`
	Max6814No2Alarm       bool      `gorm:"column:max_6814_no2_alarm;not null;default:false" json:"max_6814_no2_alarm"`
	MaxRadStat            float64   `gorm:"column:max_rad_stat;not null;default:30.0" json:"max_rad_stat"`
	MaxRadDyn             float64   `gorm:"column:max_rad_dyn;not null;default:30.0" json:"max_rad_dyn"`
	MaxRadStatAlarm       bool      `gorm:"column:max_rad_stat_alarm;not null;default:false" json:"max_rad_stat_alarm"`
	MaxRadDynAlarm        bool      `gorm:"column:max_rad_dyn_alarm;not null;default:false" json:"max_rad_dyn_alarm"`
	MaxBmx280Tempr        float64   `gorm:"column:max_bmx280_tempr;not null;default:30.0" json:"max_bmx280_tempr"`
	MinBmx280Tempr        float64   `gorm:"column:min_bmx280_tempr;not null;default:-20.0" json:"min_bmx280_tempr"`
	MaxBmx280TemprAlarm   bool      `gorm:"column:max_bmx280_tempr_alarm;not null;default:false" json:"max_bmx280_tempr_alarm"`
	MinBmx280TemprAlarm   bool      `gorm:"column:min_bmx280_tempr_alarm;not null;default:false" json:"min_bmx280_tempr_alarm"`
	RadsensHVState        bool      `gorm:"column:radsens_hv_state;not null;default:false" json:"radsens_hv_state"`
	RadsensHVMode         bool      `gorm:"column:radsens_hv_mode;not null;default:true" json:"radsens_hv_mode"`
	RadsensSensitivity    int       `gorm:"column:radsens_sensitivity;not null;default:105" json:"radsens_sensitivity"`
	RadsensSensitivitySet bool      `gorm:"column:radsens_sensitivity_set;not null;default:false" json:"radsens_sensitivity_set"`
	ClearJournalEsp32     bool      `gorm:"column:clear_journal_esp32;not null;default:false" json:"clear_journal_esp32"`
	Esp32DateTimeNow      time.Time `gorm:"column:esp32_date_time_now;not null;autoCreateTime;default:CURRENT_TIMESTAMP" json:"esp32_date_time_now"`
	UpdatedAt             time.Time `gorm:"column:updatedat;not null;autoCreateTime;default:CURRENT_TIMESTAMP" json:"date_time"`
}

func (Settings) TableName() string {
	return "settings"
}

type Esp32DateTime struct {
	Esp32DateTime    string    `gorm:"esp32_date_time" json:"esp32_date_time"`
	Esp32DateTimeNow time.Time `gorm:"esp32_date_time_now" json:"esp32_date_time_now"`
	DateTime         time.Time `gorm:"date_time" json:"date_time"`
}

type TaskParams struct {
	ID     uint32 `gorm:"column:id;not null;unique;index" json:"id"`
	Name   string `gorm:"column:name;not null;size:45" json:"name"`
	TaskID string `gorm:"column:task_id;not null;size:45" json:"task_id"`
}

func (TaskParams) TableName() string {
	return "task_params"
}

type Tasks struct {
	ID     string       `gorm:"column:id;not null;primaryKey;unique;index;size:45" json:"id"`
	Name   string       `gorm:"column:name;not null;unique;index;size:45" json:"name"`
	Note   string       `gorm:"column:note" json:"note"`
	Params []TaskParams `json:"params"`
}

func (Tasks) TableName() string {
	return "tasks"
}

type Periods struct {
	ID   string `gorm:"column:id;not null;primaryKey;unique;index;size:16" json:"id"`
	Name string `gorm:"column:name;not null;unique;index;size:45" json:"name"`
	Idx  int    `gorm:"column:idx;not null;unique;index" json:"idx"`
}

func (Periods) TableName() string {
	return "periods"
}

type Days struct {
	ID   uint32 `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Name string `gorm:"column:name;not null" json:"name"`
}

func (Days) TableName() string {
	return "days"
}

type Executors struct {
	ID string `gorm:"column:id;not null;primaryKey;unique;index;size:20" json:"id"`
}

func (Executors) TableName() string {
	return "executors"
}

type JobParams struct {
	ID    uint32 `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Name  string `gorm:"column:name;not null;size:45" json:"name"`
	Value string `gorm:"column:value;not null" json:"value"`
	JobID uint32 `gorm:"column:job_id;not null" json:"job_id"`
}

func (JobParams) TableName() string {
	return "job_params"
}

type Jobs struct {
	ID       uint32      `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Note     string      `gorm:"column:note;not null;unique" json:"note"`
	Active   int         `gorm:"column:active;not null" json:"active"`
	Value    int         `gorm:"column:value;not null" json:"value"`
	Time     string      `gorm:"column:time;size:45" json:"time"`
	Date     string      `gorm:"column:date;size:45" json:"date"`
	Verbose  int         `gorm:"column:verbose;not null" json:"verbose"`
	Executor string      `gorm:"column:executor_id;not null;size:20" json:"executor_id"`
	TaskID   string      `gorm:"colunm:task_id;not null;size:45" json:"task_id"`
	PeriodID string      `gorm:"column:period_id;not null;size:16" json:"period_id"`
	DayID    int         `gorm:"column:day_id" json:"day_id"`
	Task     Tasks       `json:"task"`
	Period   Periods     `json:"period"`
	Day      Days        `json:"day"`
	Params   []JobParams `json:"params"`
}

func (Jobs) TableName() string {
	return "jobs"
}

type SSHKeys struct {
	ID                uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Finger            string    `gorm:"column:finger;not null;type:TEXT" json:"finger"`
	Owner             string    `gorm:"column:owner;not null;unique;index;size:45" json:"owner"`
	CreatedAt         time.Time `gorm:"column:created;not null;default:CURRENT_TIMESTAMP" json:"created"`
	UpdatedAt         time.Time `gorm:"column:used" json:"used"`
	HasRecentActivity bool      `gorm:"-" sql:"activity"`
	ShortFinger       string    `gorm:"-" sql:"short_finger"`
}

func (SSHKeys) TableName() string {
	return "ssh_keys"
}

type Knowhosts struct {
	ID                uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Host              string    `gorm:"column:host;not null;unique;index;size:45" json:"host"`
	Finger            string    `gorm:"column:finger;not null;type:TEXT" json:"finger"`
	CreatedAt         time.Time `gorm:"column:created;not null;default:CURRENT_TIMESTAMP" json:"created"`
	UpdatedAt         time.Time `gorm:"column:used" json:"used"`
	HasRecentActivity bool      `gorm:"-" sql:"activity"`
	ShortFinger       string    `gorm:"-" sql:"short_finger"`
}

func (Knowhosts) TableName() string {
	return "knowhosts"
}

type GitKeys struct {
	ID                uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Finger            string    `gorm:"column:finger;not null;type:TEXT" json:"finger"`
	Owner             string    `gorm:"column:owner;not null;unique;index;size:45" json:"owner"`
	CreatedAt         time.Time `gorm:"column:created;not null;default:CURRENT_TIMESTAMP" json:"created"`
	UpdatedAt         time.Time `gorm:"column:used" json:"used"`
	HasRecentActivity bool      `gorm:"-" sql:"activity"`
	ShortFinger       string    `gorm:"-" sql:"short_finger"`
}

func (GitKeys) TableName() string {
	return "git_keys"
}

type GitUsers struct {
	ID                uint32    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Username          string    `gorm:"column:username;not null;size:45" json:"username"`
	Password          string    `gorm:"column:password;not null;size:45" json:"password"`
	Service           string    `gorm:"column:service;not null;unique;index;size:45" json:"service"`
	CreatedAt         time.Time `gorm:"column:created;not null;default:CURRENT_TIMESTAMP" json:"created"`
	UpdatedAt         time.Time `gorm:"column:used" json:"used"`
	HasRecentActivity bool      `gorm:"-" sql:"activity"`
}

func (GitUsers) TableName() string {
	return "git_users"
}

type Blocklist struct {
	ID string `gorm:"column:hostname;not null;primaryKey;unique;index;size:100" json:"id"`
}

func (Blocklist) TableName() string {
	return "blocklists"
}

type Homezone struct {
	ID         uint32 `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	DomainName string `gorm:"column:domain_name;not null;unique;index;size:100" json:"domain_name"`
	IPv4       string `gorm:"column:ip;not null;size:20" json:"ip"`
	Mac        string `gorm:"column:mac;size:20" json:"mac"`
	Note       string `gorm:"column:note" json:"note"`
	Active     bool   `json:"active"`
}

func (Homezone) TableName() string {
	return "homezones"
}

type AccesList struct {
	ID string `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
}

func (AccesList) TableName() string {
	return "access_lists"
}

type ToVpnManual struct {
	ID     uint32 `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Name   string `gorm:"column:hostname;not null;unique;index;size:100" json:"name"`
	Note   string `gorm:"column:note" json:"note"`
	ListID string `gorm:"column:list_id" json:"list_id"`
}

func (ToVpnManual) TableName() string {
	return "tovpn_manuals"
}

type ToVpnAuto struct {
	ID        string    `gorm:"column:hostname;not null;primaryKey;unique;index;size:100" json:"id"`
	CreatedAt time.Time `gorm:"column:createdat;not null;default:CURRENT_TIMESTAMP" json:"createdat"`
}

func (ToVpnAuto) TableName() string {
	return "tovpn_autos"
}

type ToVpnIgnore struct {
	ID        string    `gorm:"column:hostname;not null;primaryKey;unique;index;size:100" json:"id"`
	UpdatedAt time.Time `gorm:"column:updatedat" json:"updatedat"`
}

func (ToVpnIgnore) TableName() string {
	return "tovpn_ignores"
}

type User struct {
	ID        string    `gorm:"column:id;not null;primaryKey;unique;index" json:"id"`
	Username  string    `gorm:"column:username;not null;size:45" json:"username"`
	Password  string    `gorm:"column:password;not null;size:255" json:"password"`
	Token     string    `gorm:"column:tokenhash;not null;size:255" json:"tokenhash"`
	CreatedAt time.Time `gorm:"column:createdat;not null;default:CURRENT_TIMESTAMP" json:"createdat"`
	UpdatedAt time.Time `gorm:"column:updatedat" json:"updatedat"`
}

func (User) TableName() string {
	return "users"
}

type SyncParams struct {
	ID          uint32 `gorm:"column:id;not null;unique;index" json:"id"`
	SyncType    string `gorm:"column:sync_type;not null;size:20" json:"sync_type"`
	SyncTableID string `gorm:"column:table_id;not null" json:"table_id"`
}

func (SyncParams) TableName() string {
	return "sync_params"
}

type SyncTables struct {
	ID       string       `gorm:"column:name;not null;size:45;primaryKey;unique;index" json:"name"`
	Note     string       `gorm:"column:note;not null" json:"note"`
	SyncedAt time.Time    `gorm:"column:syncedat" json:"syncedat"`
	Params   []SyncParams `json:"params"`
}

func (SyncTables) SyncTables() string {
	return "sync_tables"
}

type ProxyState struct {
	Active     bool `json:"active"`
	BlkListOn  bool `json:"blkliston"`
	CacheOn    bool `json:"cacheon"`
	UnlockerOn bool `json:"unlockeron"`
}

type Period struct {
	Begin time.Time `json:"begin"`
	End   time.Time `json:"end"`
}

type Callback struct {
	Query  string        `json:"query"`
	Params []interface{} `json:"params"`
}

type CronJobs struct {
	Note          string
	ScheduledTime time.Time
}
