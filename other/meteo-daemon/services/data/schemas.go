package data

const schema = "CREATE SCHEMA IF NOT EXISTS ESP8266;"
const selectSchema = "USE ESP8266;"

const bmp280Schema = `
CREATE TABLE IF NOT EXISTS BMP280 (
	bmp280_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	press FLOAT NOT NULL,
	temp FLOAT NOT NULL,
	hum FLOAT NOT NULL,
	PRIMARY KEY (bmp280_id));
`
const ds18b20Schema = `
CREATE TABLE IF NOT EXISTS DS18B20 (
	ds18b20_id INT NOT NULL AUTO_INCREMENT,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	temp FLOAT NOT NULL,
	PRIMARY KEY (ds18b20_id));
`
const radsensSchema = `
CREATE TABLE IF NOT EXISTS RADSENS (
	radsens_id INT NOT NULL AUTO_INCREMENT,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	dynamic FLOAT NOT NULL,
	static FLOAT NOT NULL,
	pulse INT NOT NULL,
	PRIMARY KEY (radsens_id));
`
const ze08ch2oSchema = `
CREATE TABLE IF NOT EXISTS ZE08CH2O (
	ze08ch2o_id INT NOT NULL AUTO_INCREMENT,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	ch2o INT NOT NULL,
	PRIMARY KEY (ze08ch2o_id));
`
const logingSchema = `
	CREATE TABLE IF NOT EXISTS LOGING (
	loging_id INT NOT NULL AUTO_INCREMENT,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	message VARCHAR(128) NOT NULL,
	type VARCHAR(2) NOT NULL,
	PRIMARY KEY (loging_id));
`
const settingSchema = `
CREATE TABLE IF NOT EXISTS SETTINGS (
	settings_id INT NOT NULL AUTO_INCREMENT,
	valve_state TINYINT NOT NULL DEFAULT 0,
	CCS811_baseline INT NOT NULL DEFAULT 0,
	valve_disable TINYINT NOT NULL DEFAULT 1,
	min_temp FLOAT NOT NULL DEFAULT 10.0,
	max_temp FLOAT NOT NULL DEFAULT 12.0,
	date_time DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	firmware VARCHAR(64) NOT NULL,
	upgrade_status TINYINT NOT NULL DEFAULT 0,
	setup_mode TINYINT NOT NULL DEFAULT 0,
	setup_status TINYINT NOT NULL DEFAULT 0,
	reboot TINYINT NOT NULL DEFAULT 0,
	rebooted TINYINT NOT NULL DEFAULT 0,
	max_ch2o FLOAT NOT NULL DEFAULT 0,
	max_ch2o_alarm TINYINT NOT NULL DEFAULT 0,
	max_ds18b20 FLOAT NOT NULL,
	max_ds18b20_alarm TINYINT NOT NULL,
	min_ds18b20 FLOAT NOT NULL,
	min_ds18b20_alarm TINYINT NOT NULL,
	max_6814_nh3 FLOAT NOT NULL,
	max_6814_co FLOAT NOT NULL,
	max_6814_no2 FLOAT NOT NULL,
	max_6814_nh3_alarm TINYINT NOT NULL,
	max_6814_co_alarm TINYINT NOT NULL,
	max_6814_no2_alarm TINYINT NOT NULL,
	max_rad_stat FLOAT NOT NULL,
	max_rad_stat_alarm TINYINT NOT NULL,
	max_rad_dyn FLOAT NOT NULL,
	max_rad_dyn_alarm TINYINT NOT NULL,
	esp32_date_time_now DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	max_bmp280_tempr_alarm TINYINT NOT NULL,
	min_bmp280_tempr_alarm TINYINT NOT NULL,
	max_bmp280_tempr FLOAT NOT NULL,
	min_bmp280_tempr FLOAT NOT NULL,
	radsens_hv_state TINYINT NOT NULL,
	radsens_hv_mode TINYINT NOT NULL,
	radsens_sensitivity INT NOT NULL,
	radsens_sensitivity_set TINYINT NOT NULL,
	clear_journal_esp32 TINYINT NOT NULL,
	PRIMARY KEY (settings_id));
`
const sshkeysSchema = `
CREATE TABLE IF NOT EXISTS SSH_KEYS (
	ssh_keys_id INT NOT NULL AUTO_INCREMENT,
	finger TEXT NOT NULL,
	owner VARCHAR(64) NOT NULL,
	created DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	used DATETIME NULL,
	PRIMARY KEY (ssh_keys_id),
	UNIQUE INDEX ssh_keys_id_UNIQUE (ssh_keys_id ASC) ,
	UNIQUE INDEX owner_UNIQUE (owner ASC) );
`

const knownhostsSchema = `
CREATE TABLE IF NOT EXISTS KNOWNHOSTS (
	knownhosts_id INT NOT NULL AUTO_INCREMENT,
	host VARCHAR(45) NOT NULL,
	finger TEXT NOT NULL,
	created DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	used DATETIME NULL,
	PRIMARY KEY (knownhosts_id),
	UNIQUE INDEX knownhosts_id_UNIQUE (knownhosts_id ASC) ,
	UNIQUE INDEX host_UNIQUE (host ASC) );
`

const gitkeysSchema = `
CREATE TABLE IF NOT EXISTS GIT_KEYS (
	git_keys_id INT NOT NULL AUTO_INCREMENT,
	owner VARCHAR(64) NOT NULL,
	finger TEXT NOT NULL,
	created DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	used DATETIME NULL,
	PRIMARY KEY (git_keys_id),
	UNIQUE INDEX owner_UNIQUE (owner ASC) );
`

const gitusersSchema = `
CREATE TABLE IF NOT EXISTS GIT_USERS (
	git_users_id INT NOT NULL AUTO_INCREMENT,
	username VARCHAR(45) NOT NULL,
	password VARCHAR(45) NOT NULL,
	service VARCHAR(45) NOT NULL,
	created DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	used DATETIME NULL,
	PRIMARY KEY (git_users_id),
	UNIQUE INDEX service_UNIQUE (service ASC) );
`

const tasksSchema = `
CREATE TABLE IF NOT EXISTS TASKS (
	task_id VARCHAR(45) NOT NULL,
	name VARCHAR(45) NOT NULL,
	note VARCHAR(255) NULL,
	PRIMARY KEY (task_id),
	UNIQUE INDEX name_UNIQUE (name ASC) ,
	UNIQUE INDEX task_id_UNIQUE (task_id ASC) );
`

const periodsSchema = `
CREATE TABLE IF NOT EXISTS PERIODS (
	period_id VARCHAR(16) NOT NULL,
	name VARCHAR(45) NOT NULL,
	idx INT NOT NULL,
	PRIMARY KEY (period_id),
	UNIQUE INDEX name_UNIQUE (name ASC) );
`

const periodsInsert = `
INSERT INTO PERIODS (period_id, name, idx) VALUES('day', 'День', 1);
INSERT INTO PERIODS (period_id, name, idx) VALUES('day_of_week', 'День недели', 2);
INSERT INTO PERIODS (period_id, name, idx) VALUES('hour', 'Час', 3);
INSERT INTO PERIODS (period_id, name, idx) VALUES('min', 'Минута', 4);
INSERT INTO PERIODS (period_id, name, idx) VALUES('month', 'Месяц', 5);
INSERT INTO PERIODS (period_id, name, idx) VALUES('one', 'Единоразово', 6);
INSERT INTO PERIODS (period_id, name, idx) VALUES('sec', 'Секунда', 7);
INSERT INTO PERIODS (period_id, name, idx) VALUES('week', 'Неделя', 8);
`

const daysSchema = `
CREATE TABLE IF NOT EXISTS DAYS (
	day_id TINYINT NOT NULL,
	name VARCHAR(45) NOT NULL,
	PRIMARY KEY (day_id));
`

const daysInsert = `
INSERT INTO DAYS (day_id, name) VALUES(1, 'Понедельник');
INSERT INTO DAYS (day_id, name) VALUES(2, 'Вторник');
INSERT INTO DAYS (day_id, name) VALUES(3, 'Вторник');
INSERT INTO DAYS (day_id, name) VALUES(4, 'Четверг');
INSERT INTO DAYS (day_id, name) VALUES(5, 'Пятницу');
INSERT INTO DAYS (day_id, name) VALUES(6, 'Субботу');
INSERT INTO DAYS (day_id, name) VALUES(7, 'Воскресенье');
`

const jobsSchema = `
CREATE TABLE IF NOT EXISTS JOBS (
	job_id INT NOT NULL AUTO_INCREMENT,
	active TINYINT NOT NULL,
	value INT NOT NULL,
	time TIME NULL,
	date DATE NULL,
	task_id VARCHAR(45) NOT NULL,
	period_id VARCHAR(16) NOT NULL,
	day_id TINYINT NULL,
	PRIMARY KEY (job_id),
	INDEX fk_JOBS_TASK_idx (task_id ASC) ,
	INDEX fk_JOBS_PERIOD1_idx (period_id ASC) ,
	INDEX fk_JOBS_DAYS1_idx (day_id ASC) ,
	CONSTRAINT fk_JOBS_TASK
	  FOREIGN KEY (task_id)
	  REFERENCES TASKS (task_id)
	  ON DELETE NO ACTION
	  ON UPDATE NO ACTION,
	CONSTRAINT fk_JOBS_PERIOD1
	  FOREIGN KEY (period_id)
	  REFERENCES PERIODS (period_id)
	  ON DELETE NO ACTION
	  ON UPDATE NO ACTION,
	CONSTRAINT fk_JOBS_DAYS1
	  FOREIGN KEY (day_id)
	  REFERENCES sDAYS (day_id)
	  ON DELETE NO ACTION
	  ON UPDATE NO ACTION);
`

const jobparamsSchema = `
CREATE TABLE IF NOT EXISTS JOB_PARAMS (
	job_params_id INT NOT NULL AUTO_INCREMENT,
	name VARCHAR(45) NOT NULL,
	value TEXT NOT NULL,
	job_id INT NOT NULL,
	PRIMARY KEY (job_params_id),
	INDEX fk_JOB_PARAMS_JOBS1_idx (job_id ASC) ,
	CONSTRAINT fk_JOB_PARAMS_JOBS1
	  FOREIGN KEY (job_id)
	  REFERENCES JOBS (job_id)
	  ON DELETE CASCADE
	  ON UPDATE CASCADE);
`

const taskparamsSchema = `
CREATE TABLE IF NOT EXISTS TASK_PARAMS (
	task_params_id INT NOT NULL AUTO_INCREMENT,
	name VARCHAR(45) NULL,
	task_id VARCHAR(45) NOT NULL,
	PRIMARY KEY (task_params_id),
	INDEX fk_TASK_PARAMS_TASKS1_idx (task_id ASC) ,
	CONSTRAINT fk_TASK_PARAMS_TASKS1
	  FOREIGN KEY (task_id)
	  REFERENCES ETASKS (task_id)
	  ON DELETE CASCADE
	  ON UPDATE CASCADE);
`

const usersSchema = `
CREATE TABLE IF NOT EXISTS USERS (
	id VARCHAR(36) NOT NULL,
	username VARCHAR(255) NULL,
	password VARCHAR(255) NOT NULL,
	tokenhash VARCHAR(15) NOT NULL,
	createdat TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
	updatedat TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
	PRIMARY KEY (id));
`

const blocklistSchema = `
CREATE TABLE IF NOT EXISTS DNS_BLOCKLIST (
	hostname VARCHAR(64) NOT NULL,
	PRIMARY KEY (hostname),
	UNIQUE INDEX hostname_UNIQUE (hostname ASC) );
`

const alljobsSchema = `
CREATE  OR REPLACE VIEW ALL_JOBS AS
SELECT j.job_id, j.task_id, j.period_id, j.value,  
IF(j.time IS NULL, "", j.time) AS time, 
IF(j.date IS NULL, "", j.date) AS date,
j.active, 
IF(j.day_id IS NULL, 0, j.day_id) AS day_id,
t.name AS task, p.name AS period, 
IF((SELECT name FROM DAYS WHERE day_id = j.day_id) IS NULL, "", 
(SELECT name FROM DAYS WHERE day_id = j.day_id))  AS day
FROM JOBS j, TASKS t, PERIODS p 
WHERE j.task_id = t.task_id AND j.period_id = p.period_id;
`

const tovpnmanualSchema = `
CREATE TABLE IF NOT EXISTS TO_VPN_MANUAL (
	hostname VARCHAR(100) NOT NULL,
	note VARCHAR(255) NULL,
	PRIMARY KEY (hostname),
	UNIQUE INDEX hostname_UNIQUE (hostname ASC) );
`

const tovpnautoSchema = `
CREATE TABLE IF NOT EXISTS TO_VPN_AUTO (
	hostname VARCHAR(100) NOT NULL,
	createdat DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
	PRIMARY KEY (hostname),
	UNIQUE INDEX hostname_UNIQUE (hostname ASC) );
`

const tovpnignoreSchema = `
CREATE TABLE IF NOT EXISTS TO_VPN_IGNORE (
	hostname VARCHAR(100) NOT NULL,
	updatedat TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
	PRIMARY KEY (hostname),
	UNIQUE INDEX hostname_UNIQUE (hostname ASC) );
`
const synctablesSchema = `
CREATE TABLE IF NOT EXISTS SYNC_TABLES (
	id INT NOT NULL,
	name VARCHAR(45) NOT NULL,
	PRIMARY KEY (id),
	UNIQUE INDEX name_UNIQUE (name ASC) );
`
const synctablesInsert = `
INSERT INTO SYNC_TABLES (id, name) VALUE(1, 'DAYS');
INSERT INTO SYNC_TABLES (id, name) VALUE(2, 'PERIODS');
INSERT INTO SYNC_TABLES (id, name) VALUE(3, 'TASKS');
INSERT INTO SYNC_TABLES (id, name) VALUE(4, 'TASKS_PARAMS');
INSERT INTO SYNC_TABLES (id, name) VALUE(5, 'JOBS');
INSERT INTO SYNC_TABLES (id, name) VALUE(6, 'JOBS_PARAMS');
INSERT INTO SYNC_TABLES (id, name) VALUE(7, 'USERS');
INSERT INTO SYNC_TABLES (id, name) VALUE(8, 'SSH_KEYS');
INSERT INTO SYNC_TABLES (id, name) VALUE(9, 'KNOWNHOSTS');
INSERT INTO SYNC_TABLES (id, name) VALUE(10, 'GIT_KEYS');
INSERT INTO SYNC_TABLES (id, name) VALUE(11, 'GIT_USERS');
INSERT INTO SYNC_TABLES (id, name) VALUE(12, 'TO_VPN_MANUAL');
INSERT INTO SYNC_TABLES (id, name) VALUE(13, 'TO_VPN_AUTO');
INSERT INTO SYNC_TABLES (id, name) VALUE(14, 'TO_VPN_IGNORE');
INSERT INTO SYNC_TABLES (id, name) VALUE(15, 'BMP280');
INSERT INTO SYNC_TABLES (id, name) VALUE(16, 'DS18B20');
INSERT INTO SYNC_TABLES (id, name) VALUE(17, 'RADSENS');
INSERT INTO SYNC_TABLES (id, name) VALUE(18, 'SETTINGS');
INSERT INTO SYNC_TABLES (id, name) VALUE(19, 'LOGING');
INSERT INTO SYNC_TABLES (id, name) VALUE(20, 'ZE08CH2O');
`
