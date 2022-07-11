-- MySQL Workbench Synchronization
-- Generated: 2022-06-16 07:27
-- Model: New Model
-- Version: 1.0
-- Project: Name of the project
-- Author: Unknown

SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';

ALTER SCHEMA `ESP8266`  DEFAULT CHARACTER SET utf8  DEFAULT COLLATE utf8_general_ci ;

ALTER TABLE `ESP8266`.`bmx280` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `press` `press` FLOAT(11) NOT NULL ,
CHANGE COLUMN `tempr` `tempr` FLOAT(11) NOT NULL ,
CHANGE COLUMN `hum` `hum` FLOAT(11) NOT NULL ,
DROP INDEX `idx_bmx280_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`ds18b20` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `tempr` `tempr` FLOAT(11) NOT NULL ,
DROP INDEX `idx_ds18b20_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`radsens` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `dynamic` `dynamic` FLOAT(11) NOT NULL ,
CHANGE COLUMN `static` `static` FLOAT(11) NOT NULL ,
DROP INDEX `idx_radsens_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`ze08ch2o` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
DROP INDEX `idx_ze08ch2o_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`logging` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
DROP INDEX `idx_logging_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`settings` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `ccs811_baseline` `ccs811_baseline` INT(11) NOT NULL DEFAULT 0 AFTER `valve_state`,
CHANGE COLUMN `updatedat` `updatedat` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `max_temp`,
CHANGE COLUMN `max_rad_stat_alarm` `max_rad_stat_alarm` TINYINT(4) NOT NULL DEFAULT 0 AFTER `max_rad_stat`,
CHANGE COLUMN `esp32_date_time_now` `esp32_date_time_now` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `max_rad_dyn_alarm`,
CHANGE COLUMN `max_bmx280_tempr_alarm` `max_bmx280_tempr_alarm` TINYINT(4) NOT NULL DEFAULT 0 AFTER `esp32_date_time_now`,
CHANGE COLUMN `min_bmx280_tempr_alarm` `min_bmx280_tempr_alarm` TINYINT(4) NOT NULL DEFAULT 0 AFTER `max_bmx280_tempr_alarm`,
CHANGE COLUMN `min_ds18b20` `min_ds18b20` FLOAT(11) NOT NULL DEFAULT 8 AFTER `min_bmx280_tempr`,
CHANGE COLUMN `min_ds18b20_alarm` `min_ds18b20_alarm` TINYINT(4) NOT NULL DEFAULT 0 AFTER `min_ds18b20`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL ,
CHANGE COLUMN `valve_state` `valve_state` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `valve_disable` `valve_disable` TINYINT(4) NOT NULL DEFAULT 1 ,
CHANGE COLUMN `min_temp` `min_temp` FLOAT(11) NOT NULL DEFAULT 10.0 ,
CHANGE COLUMN `max_temp` `max_temp` FLOAT(11) NOT NULL DEFAULT 12.0 ,
CHANGE COLUMN `firmware` `firmware` VARCHAR(64) NOT NULL DEFAULT '_EMPTY_' ,
CHANGE COLUMN `upgrade_status` `upgrade_status` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `setup_mode` `setup_mode` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `setup_status` `setup_status` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `reboot` `reboot` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `rebooted` `rebooted` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_ch2o` `max_ch2o` FLOAT(11) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_ch2o_alarm` `max_ch2o_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_ds18b20` `max_ds18b20` FLOAT(11) NOT NULL DEFAULT 30 ,
CHANGE COLUMN `max_ds18b20_alarm` `max_ds18b20_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_nh3` `max_6814_nh3` FLOAT(11) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_co` `max_6814_co` FLOAT(11) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_no2` `max_6814_no2` FLOAT(11) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_nh3_alarm` `max_6814_nh3_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_co_alarm` `max_6814_co_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_6814_no2_alarm` `max_6814_no2_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_rad_stat` `max_rad_stat` FLOAT(11) NOT NULL DEFAULT 30 ,
CHANGE COLUMN `max_rad_dyn` `max_rad_dyn` FLOAT(11) NOT NULL DEFAULT 30 ,
CHANGE COLUMN `max_rad_dyn_alarm` `max_rad_dyn_alarm` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `max_bmx280_tempr` `max_bmx280_tempr` FLOAT(11) NOT NULL DEFAULT 30 ,
CHANGE COLUMN `min_bmx280_tempr` `min_bmx280_tempr` FLOAT(11) NOT NULL DEFAULT -20 ,
CHANGE COLUMN `radsens_hv_state` `radsens_hv_state` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `radsens_hv_mode` `radsens_hv_mode` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `radsens_sensitivity_set` `radsens_sensitivity_set` TINYINT(4) NOT NULL DEFAULT 0 ,
CHANGE COLUMN `clear_journal_esp32` `clear_journal_esp32` TINYINT(4) NOT NULL DEFAULT 0 ,
DROP INDEX `id` ;
;

CREATE TABLE IF NOT EXISTS `ESP8266`.`MICS5524` (
  `mics5524_id` INT(11) NOT NULL AUTO_INCREMENT,
  `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `Ao` FLOAT(11) NOT NULL,
  `En` FLOAT(11) NOT NULL,
  PRIMARY KEY (`mics5524_id`))
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

ALTER TABLE `ESP8266`.`mics6814` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `date_time` `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `no2` `no2` FLOAT(11) NOT NULL ,
CHANGE COLUMN `nh3` `nh3` FLOAT(11) NOT NULL ,
CHANGE COLUMN `co` `co` FLOAT(11) NOT NULL ,
DROP INDEX `idx_mics6814_id` ,
DROP INDEX `id` ;
;

CREATE TABLE IF NOT EXISTS `ESP8266`.`CCS811` (
  `ccs811_id` INT(11) NOT NULL AUTO_INCREMENT,
  `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `CO2` FLOAT(11) NOT NULL,
  `TVOC` FLOAT(11) NOT NULL,
  PRIMARY KEY (`ccs811_id`))
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

CREATE TABLE IF NOT EXISTS `ESP8266`.`SGP30` (
  `sgp30_id` INT(11) NOT NULL AUTO_INCREMENT,
  `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `eCO2` FLOAT(11) NOT NULL,
  `TVOC` FLOAT(11) NOT NULL,
  PRIMARY KEY (`sgp30_id`))
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

CREATE TABLE IF NOT EXISTS `ESP8266`.`speedtest` (
  `idspeedtest` INT(11) NOT NULL AUTO_INCREMENT,
  `date_time` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `source` VARCHAR(45) NOT NULL,
  `receiver` FLOAT(11) NULL DEFAULT NULL,
  `sender` FLOAT(11) NULL DEFAULT NULL,
  `response` INT(11) NULL DEFAULT NULL,
  PRIMARY KEY (`idspeedtest`),
  UNIQUE INDEX `idspeedtest_UNIQUE` (`idspeedtest` ASC) VISIBLE)
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

ALTER TABLE `ESP8266`.`ssh_keys` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `owner` `owner` VARCHAR(64) NOT NULL ,
ADD UNIQUE INDEX `ssh_keys_id_UNIQUE` (`id` ASC) VISIBLE,
ADD UNIQUE INDEX `owner_UNIQUE` (`owner` ASC) VISIBLE,
DROP INDEX `idx_ssh_keys_id` ,
DROP INDEX `idx_ssh_keys_owner` ,
DROP INDEX `owner` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`knowhosts` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
DROP COLUMN `short_finger`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
ADD UNIQUE INDEX `knownhosts_id_UNIQUE` (`id` ASC) VISIBLE,
ADD UNIQUE INDEX `host_UNIQUE` (`host` ASC) VISIBLE,
DROP INDEX `idx_knowhosts_host` ,
DROP INDEX `idx_knowhosts_id` ,
DROP INDEX `host` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`git_keys` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `owner` `owner` VARCHAR(64) NOT NULL AFTER `id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
ADD UNIQUE INDEX `owner_UNIQUE` (`owner` ASC) VISIBLE,
DROP INDEX `idx_git_keys_owner` ,
DROP INDEX `idx_git_keys_id` ,
DROP INDEX `owner` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`git_users` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
ADD UNIQUE INDEX `service_UNIQUE` (`service` ASC) VISIBLE,
DROP INDEX `idx_git_users_service` ,
DROP INDEX `idx_git_users_id` ,
DROP INDEX `service` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`tasks` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
ADD UNIQUE INDEX `name_UNIQUE` (`name` ASC) VISIBLE,
ADD UNIQUE INDEX `task_id_UNIQUE` (`id` ASC) VISIBLE,
DROP INDEX `idx_tasks_name` ,
DROP INDEX `idx_tasks_id` ,
DROP INDEX `name` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`periods` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
ADD UNIQUE INDEX `name_UNIQUE` (`name` ASC) VISIBLE,
ADD UNIQUE INDEX `idx_UNIQUE` (`idx` ASC) VISIBLE,
DROP INDEX `idx_periods_id` ,
DROP INDEX `idx_periods_idx` ,
DROP INDEX `idx_periods_name` ,
DROP INDEX `idx` ,
DROP INDEX `name` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`days` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` TINYINT(4) NOT NULL ,
CHANGE COLUMN `name` `name` VARCHAR(45) NOT NULL ,
DROP INDEX `idx_days_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`jobs` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `executor_id` `executor_id` VARCHAR(20) NOT NULL AFTER `day_id`,
CHANGE COLUMN `verbose` `verbose` TINYINT(4) NOT NULL AFTER `executor_id`,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `active` `active` TINYINT(4) NOT NULL ,
CHANGE COLUMN `day_id` `day_id` TINYINT(4) NULL DEFAULT NULL ,
ADD INDEX `fk_JOBS_TASK_idx` (`task_id` ASC) VISIBLE,
ADD INDEX `fk_JOBS_PERIOD1_idx` (`period_id` ASC) VISIBLE,
ADD INDEX `fk_JOBS_DAYS1_idx` (`day_id` ASC) VISIBLE,
ADD UNIQUE INDEX `note_UNIQUE` (`note` ASC) VISIBLE,
ADD INDEX `fk_jobs_executor1_idx` (`executor_id` ASC) VISIBLE,
DROP INDEX `idx_jobs_id` ,
DROP INDEX `note` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`job_params` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `value` `value` TEXT NOT NULL ,
ADD INDEX `fk_JOB_PARAMS_JOBS1_idx` (`job_id` ASC) VISIBLE,
DROP INDEX `idx_job_params_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`task_params` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
CHANGE COLUMN `name` `name` VARCHAR(45) NULL DEFAULT NULL ,
ADD INDEX `fk_TASK_PARAMS_TASKS1_idx` (`task_id` ASC) VISIBLE,
DROP INDEX `idx_task_params_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`users` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` VARCHAR(36) NOT NULL ,
CHANGE COLUMN `username` `username` VARCHAR(255) NOT NULL ,
CHANGE COLUMN `password` `password` VARCHAR(255) NOT NULL ,
CHANGE COLUMN `tokenhash` `tokenhash` VARCHAR(15) NOT NULL ,
CHANGE COLUMN `createdat` `createdat` TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ,
CHANGE COLUMN `updatedat` `updatedat` TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ,
DROP INDEX `idx_users_id` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`blocklists` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
ADD UNIQUE INDEX `hostname_UNIQUE` (`hostname` ASC) VISIBLE,
ADD PRIMARY KEY (`hostname`),
DROP INDEX `idx_blocklists_hostname` ,
DROP INDEX `hostname` ;
;

ALTER TABLE `ESP8266`.`tovpn_manuals` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
CHANGE COLUMN `id` `id` INT(11) NOT NULL AUTO_INCREMENT ,
ADD UNIQUE INDEX `hostname_UNIQUE` (`hostname` ASC) VISIBLE,
ADD UNIQUE INDEX `id_UNIQUE` (`id` ASC) VISIBLE,
DROP INDEX `idx_tovpn_manuals_hostname` ,
DROP INDEX `idx_tovpn_manuals_id` ,
DROP INDEX `hostname` ,
DROP INDEX `id` ;
;

ALTER TABLE `ESP8266`.`tovpn_autos` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
ADD UNIQUE INDEX `hostname_UNIQUE` (`hostname` ASC) VISIBLE,
ADD PRIMARY KEY (`hostname`),
DROP INDEX `idx_tovpn_autos_hostname` ,
DROP INDEX `hostname` ;
;

ALTER TABLE `ESP8266`.`tovpn_ignores` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
ADD UNIQUE INDEX `hostname_UNIQUE` (`hostname` ASC) VISIBLE,
ADD PRIMARY KEY (`hostname`),
DROP INDEX `idx_tovpn_ignores_hostname` ,
DROP INDEX `hostname` ;
;

ALTER TABLE `ESP8266`.`sync_tables` 
CHARACTER SET = utf8 , COLLATE = utf8_general_ci ,
DROP COLUMN `updatedat`,
DROP COLUMN `note`,
ADD COLUMN `id` INT(11) NOT NULL FIRST,
ADD COLUMN `list_id` VARCHAR(45) NOT NULL AFTER `syncedat`,
ADD PRIMARY KEY (`id`),
ADD UNIQUE INDEX `name_UNIQUE` (`name` ASC) VISIBLE,
ADD INDEX `fk_sync_tables_access_lists1_idx` (`list_id` ASC) VISIBLE,
DROP INDEX `idx_sync_tables_name` ,
DROP INDEX `name` ;
;

CREATE TABLE IF NOT EXISTS `ESP8266`.`executor` (
  `id` VARCHAR(20) NOT NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

CREATE TABLE IF NOT EXISTS `ESP8266`.`access_lists` (
  `id` VARCHAR(45) NOT NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB
DEFAULT CHARACTER SET = utf8;

DROP TABLE IF EXISTS `ESP8266`.`sync_types` ;

DROP TABLE IF EXISTS `ESP8266`.`sync_params` ;

DROP TABLE IF EXISTS `ESP8266`.`homezones` ;

DROP TABLE IF EXISTS `ESP8266`.`executors` ;

ALTER TABLE `ESP8266`.`jobs` 
ADD CONSTRAINT `fk_JOBS_TASK`
  FOREIGN KEY (`task_id`)
  REFERENCES `ESP8266`.`tasks` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION,
ADD CONSTRAINT `fk_JOBS_PERIOD1`
  FOREIGN KEY (`period_id`)
  REFERENCES `ESP8266`.`periods` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION,
ADD CONSTRAINT `fk_JOBS_DAYS1`
  FOREIGN KEY (`day_id`)
  REFERENCES `ESP8266`.`days` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION,
ADD CONSTRAINT `fk_jobs_executor1`
  FOREIGN KEY (`executor_id`)
  REFERENCES `ESP8266`.`executor` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION;

ALTER TABLE `ESP8266`.`job_params` 
ADD CONSTRAINT `JobsRefer`
  FOREIGN KEY (`job_id`)
  REFERENCES `ESP8266`.`jobs` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION;

ALTER TABLE `ESP8266`.`task_params` 
ADD CONSTRAINT `TasksRefer`
  FOREIGN KEY (`task_id`)
  REFERENCES `ESP8266`.`tasks` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION;

ALTER TABLE `ESP8266`.`sync_tables` 
ADD CONSTRAINT `fk_sync_tables_access_lists1`
  FOREIGN KEY (`list_id`)
  REFERENCES `ESP8266`.`access_lists` (`id`)
  ON DELETE NO ACTION
  ON UPDATE NO ACTION;


SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;
