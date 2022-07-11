package data

import (
	"context"
)

type Repository interface {
	Initialize(ctx context.Context) error

	/*-------------------Sync db-------------------*/
	AddSyncTable(ctx context.Context, table SyncTables) error
	EditSyncTable(ctx context.Context, table SyncTables, id string) error
	DelSyncTable(ctx context.Context, id string) error
	GetSyncTable(ctx context.Context, id string) (*SyncTables, error)
	GetAllSyncTables(ctx context.Context) ([]SyncTables, error)
	GetSyncTableParams(ctx context.Context, table_id string) ([]SyncParams, error)
	ExecRaw(ctx context.Context, cb Callback) error
	GetNotSyncBmx280(ctx context.Context) ([]Bmx280, error)
	AddSyncBmx280(ctx context.Context, bmx280 []Bmx280) error
	SyncBmx280(ctx context.Context) error
	ReplaceBmx280(ctx context.Context, readings []Bmx280) error
	LockBmx280(ext bool) error
	UnlockBmx280(ext bool) error
	GetNotSyncDs18b20(ctx context.Context) ([]Ds18b20, error)
	AddSyncDs18b20(ctx context.Context, ds18b20 []Ds18b20) error
	SyncDs18b20(ctx context.Context) error
	ReplaceDs18b20(ctx context.Context, readings []Ds18b20) error
	LockDs18b20(ext bool) error
	UnlockDs18b20(ext bool) error
	GetNotSyncZe08ch2o(ctx context.Context) ([]Ze08ch2o, error)
	AddSyncZe08ch2o(ctx context.Context, ze08ch2o []Ze08ch2o) error
	SyncZe08ch2o(ctx context.Context) error
	ReplaceZe08ch2o(ctx context.Context, readings []Ze08ch2o) error
	LockZe08ch2o(ext bool) error
	UnlockZe08ch2o(ext bool) error
	GetNotSyncRadsens(ctx context.Context) ([]Radsens, error)
	AddSyncRadsens(ctx context.Context, radsens []Radsens) error
	SyncRadsens(ctx context.Context) error
	ReplaceRadsens(ctx context.Context, readings []Radsens) error
	LockRadsens(ext bool) error
	UnlockRadsens(ext bool) error
	ReplaceToVpnManual(ctx context.Context, hosts []ToVpnManual) error
	ReplaceToVpnAuto(ctx context.Context, hosts []ToVpnAuto) error
	ReplaceToVpnIgnore(ctx context.Context, hosts []ToVpnIgnore) error
	ReplaceHomezones(ctx context.Context, hosts []Homezone) error
	ReplaceSshKeys(ctx context.Context, keys []SSHKeys) error
	ReplaceGitKeys(ctx context.Context, keys []GitKeys) error
	ReplaceKnownhosts(ctx context.Context, hosts []Knowhosts) error
	ReplaceGitUsers(ctx context.Context, users []GitUsers) error
	ReplaceTasks(ctx context.Context, tasks []Tasks) error
	ReplaceJobs(ctx context.Context, jobs []Jobs) error
	ReplaceExtBmx280(ctx context.Context) error
	ReplaceExtDs18b20(ctx context.Context) error
	ReplaceExtZe08ch2o(ctx context.Context) error
	ReplaceExtRadsens(ctx context.Context) error
	ReplaceExtHomeZones(ctx context.Context) error
	ReplaceExtToVpnManual(ctx context.Context) error
	ReplaceExtToVpnAuto(ctx context.Context) error
	ReplaceExtToVpnIgnore(ctx context.Context) error
	ReplaceExtSshKeys(ctx context.Context) error
	ReplaceExtGitKeys(ctx context.Context) error
	ReplaceExtGitUsers(ctx context.Context) error
	ReplaceExtTasks(ctx context.Context) error
	ReplaceExtJobs(ctx context.Context) error
	/*-------------------Sync db-------------------*/

	/*-------------------Home page-------------------*/
	GetDs18b20(ctx context.Context) (*Ds18b20, error)
	GetBmx280(ctx context.Context) (*Bmx280, error)
	GetMics6814(ctx context.Context) (*Mics6814, error)
	GetRadsens(ctx context.Context) (*Radsens, error)
	GetZe08ch2o(ctx context.Context) (*Ze08ch2o, error)
	GetEsp32DateTime(ctx context.Context) (*Esp32DateTime, error)
	CheckBmx280Tempr(ctx context.Context) error
	CheckDs18b20(ctx context.Context) error
	CheckZe08ch2o(ctx context.Context) error
	CheckRadsensDyn(ctx context.Context) error
	CheckRadsensStat(ctx context.Context) error
	SetRadsensHV(ctx context.Context, hv bool) error
	/*-------------------Home page-------------------*/

	/*-------------------Bmx280 page-------------------*/
	GetBmx280PerDayAvg(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerDayMin(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerDayMax(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerWeekAvg(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerWeekMin(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerWeekMax(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerMonthAvg(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerMonthMin(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerMonthMax(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerYearAvg(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerYearMin(ctx context.Context, p Period) ([]Bmx280, error)
	GetBmx280PerYearMax(ctx context.Context, p Period) ([]Bmx280, error)
	/*-------------------Bmx280 page-------------------*/

	/*-------------------Radsens page-------------------*/
	GetRadsensPerDay(ctx context.Context, p Period) ([]Radsens, error)
	GetRadsensPerWeek(ctx context.Context, p Period) ([]Radsens, error)
	GetRadsensPerMonth(ctx context.Context, p Period) ([]Radsens, error)
	GetRadsensPerYear(ctx context.Context, p Period) ([]Radsens, error)
	/*-------------------Radsens page-------------------*/

	/*-------------------Ds18b20 page-------------------*/
	GetDs18b20PerDay(ctx context.Context, p Period) ([]Ds18b20, error)
	GetDs18b20PerWeek(ctx context.Context, p Period) ([]Ds18b20, error)
	GetDs18b20PerMonth(ctx context.Context, p Period) ([]Ds18b20, error)
	GetDs18b20PerYear(ctx context.Context, p Period) ([]Ds18b20, error)
	/*-------------------Ds18b20 page-------------------*/

	/*-------------------Ze08ch2o page-------------------*/
	GetZe08ch2oPerDay(ctx context.Context, p Period) ([]Ze08ch2o, error)
	GetZe08ch2oPerWeek(ctx context.Context, p Period) ([]Ze08ch2o, error)
	GetZe08ch2oPerMonth(ctx context.Context, p Period) ([]Ze08ch2o, error)
	GetZe08ch2oPerYear(ctx context.Context, p Period) ([]Ze08ch2o, error)
	/*-------------------Ze08ch2o page-------------------*/

	/*-------------------ESP32-------------------*/
	AddLoging(ctx context.Context, msg, t, dts interface{}) error
	AddDs18b20(ctx context.Context, tempr interface{}) error
	AddBmx280(ctx context.Context, press, tempr, hum interface{}) error
	AddRadsens(ctx context.Context, dyn, stat, pulse interface{}) error
	AddZe08ch2o(ctx context.Context, ch2o interface{}) error
	GetEsp32Settings(ctx context.Context, dt interface{}) (*Settings, error)
	SetHVRadsens(ctx context.Context, state interface{}) error
	SetSensRadsens(ctx context.Context, sens interface{}) error
	SetAccesPointMode(ctx context.Context) error
	Esp32Reboot(ctx context.Context) error
	TerminateUpgrade(ctx context.Context) error
	SuccessUpgrade(ctx context.Context) error
	JournaCleared(ctx context.Context) error
	/*-------------------ESP32-------------------*/

	/*-------------------Loging-------------------*/
	GetLoging(ctx context.Context, p Period) ([]Logging, error)
	JournalClear(ctx context.Context) error
	/*-------------------Loging-------------------*/

	/*-------------------Setting-------------------*/
	GetSettings(ctx context.Context) (*Settings, error)
	SetSettings(ctx context.Context, s *Settings) error
	/*-------------------Setting-------------------*/

	/*-------------------Controller-------------------*/
	StartUpgradeEsp32(ctx context.Context, fName string) error
	GetUpgradeStatus(ctx context.Context) (*Settings, error)
	StartAccesPointMode(ctx context.Context) error
	GetStatusAccesPoint(ctx context.Context) (*Settings, error)
	StartRebootEsp32(ctx context.Context) error
	GetRebootStatus(ctx context.Context) (*Settings, error)
	/*-------------------Controller-------------------*/

	/*-------------------Security setup-------------------*/
	AddSshKey(ctx context.Context, sshKeys SSHKeys) error
	DelSshKey(ctx context.Context, id uint32) error
	DelKnowhost(ctx context.Context, id uint32) error
	GetAllSshKeys(ctx context.Context) ([]SSHKeys, error)
	AddGitKey(ctx context.Context, gitKeys GitKeys) error
	DelGitKey(ctx context.Context, id uint32) error
	GetAllGitKeys(ctx context.Context) ([]GitKeys, error)
	AddGitUser(ctx context.Context, gitUsers GitUsers) error
	DelGitUser(ctx context.Context, id uint32) error
	GetAllGitUsers(ctx context.Context) ([]GitUsers, error)
	AddKnowhost(ctx context.Context, host Knowhosts) error
	GetAllKnowhosts(ctx context.Context) ([]Knowhosts, error)
	UpTimeKnowhosts(ctx context.Context, host string) error
	GetKeyGitByOwner(ctx context.Context, owner string) (*GitKeys, error)
	UpTimeGitKeys(ctx context.Context, owner string) error
	UpTimeGitUsers(ctx context.Context, service string) error
	GetUserKeyByService(ctx context.Context, service string) (*GitUsers, error)
	/*-------------------Security setup-------------------*/

	/*-------------------Scheduler-------------------*/
	AddJob(ctx context.Context, job Jobs) error
	EditJob(ctx context.Context, job Jobs) error
	ActivateJob(ctx context.Context, id uint32) error
	DeactivateJob(ctx context.Context, id uint32, off bool) error
	RunJob(ctx context.Context, id uint32) error
	DelJob(ctx context.Context, id uint32) error
	GetJob(ctx context.Context, id uint32) (*Jobs, error)
	GetAllJobs(ctx context.Context) ([]Jobs, error)
	GetAllActiveJobs(ctx context.Context) ([]Jobs, error)
	GetJobParams(ctx context.Context, id uint32) ([]JobParams, error)
	GetJobParamsById(ctx context.Context, id uint32) ([]JobParams, error)
	GetAllJobParams(ctx context.Context) ([]JobParams, error)
	AddTask(ctx context.Context, task Tasks) error
	EditTask(ctx context.Context, task Tasks) error
	DelTask(ctx context.Context, id string) error
	GetTask(ctx context.Context, id string) (*Tasks, error)
	GetAllTasks(ctx context.Context) ([]Tasks, error)
	GetTaskParams(ctx context.Context, task_id string) ([]TaskParams, error)
	GetAllPeriods(ctx context.Context) ([]Periods, error)
	GetAllDays(ctx context.Context) ([]Days, error)
	GetAllExecutors(ctx context.Context) ([]Executors, error)
	/*-------------------Scheduler-------------------*/

	/*-------------------Proxy-------------------*/
	AddManualToVpn(ctx context.Context, host ToVpnManual) error
	EditManualToVpn(ctx context.Context, host ToVpnManual) error
	DelManualToVpn(ctx context.Context, id uint32) error
	GetManualToVpn(ctx context.Context, id uint32) (*ToVpnManual, error)
	GetManualToVpnByName(ctx context.Context, name string) (ToVpnManual, error)
	GetAllAccessLists(ctx context.Context) ([]AccesList, error)
	GetAllManualToVpn(ctx context.Context) ([]ToVpnManual, error)
	AddAutoToVpn(ctx context.Context, host ToVpnAuto) error
	DelAutoToVpn(ctx context.Context, id string) error
	RestoreAutoToVpn(ctx context.Context, id string) error
	GetAutoToVpn(ctx context.Context, id string) (*ToVpnAuto, error)
	GetAllAutoToVpn(ctx context.Context) ([]ToVpnAuto, error)
	AddIgnoreToVpn(ctx context.Context, id string) error
	DelIgnoreToVpn(ctx context.Context, id string) error
	GetAllIgnore(ctx context.Context) ([]ToVpnIgnore, error)
	AddBlockHost(ctx context.Context, host Blocklist) error
	ClearBlocklist(ctx context.Context) error
	GetAllBlockHosts(ctx context.Context) ([]Blocklist, error)
	AddHomeZoneHost(ctx context.Context, host Homezone) error
	EditHomeZoneHost(ctx context.Context, host Homezone) error
	DelHomeZoneHost(ctx context.Context, id uint32) error
	GetHomeZoneHost(ctx context.Context, id uint32) (*Homezone, error)
	GetAllHomeZoneHosts(ctx context.Context) ([]Homezone, error)
	/*-------------------Proxy-------------------*/

	/*-------------------Auth-------------------*/
	Create(ctx context.Context, user *User) error
	GetUserByID(ctx context.Context, userID string) (*User, error)
	UpdateUsername(ctx context.Context, user *User) error
	UpdatePassword(ctx context.Context, userID string, password string, tokenHash string) error
	/*-------------------Auth-------------------*/
}
