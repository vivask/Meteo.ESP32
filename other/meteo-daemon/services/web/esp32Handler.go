package web

import (
	"context"
	"encoding/json"
	"io/ioutil"
	"net/http"
	"os"
	s "strings"

	"github.com/golang/gddo/httputil/header"
)

const (
	WATCHDOG_FIRMWARE_RETRY = 60
	WATCHDOG_FIRMWARE_TIMER = 2
)

var firmwareFile string

func (web *Web) uploadHandler(w http.ResponseWriter, r *http.Request) {
	r.ParseMultipartForm(10 << 20)
	file, handler, err := r.FormFile("uploadfile")
	if err != nil {
		web.logger.Error("Error Retrieving the File")
		web.logger.Error(err)
		return
	}
	defer file.Close()

	web.logger.Infof("Uploaded File: %+v, Size: %+v\n", handler.Filename, handler.Size)

	byteArray, err := ioutil.ReadAll(file)
	if err != nil {
		web.logger.Error(err)
	}

	fileName := web.workdir + "/" + web.config.UploadPath + "/" + handler.Filename
	err = ioutil.WriteFile(fileName, byteArray, 0644)
	if err != nil {
		web.logger.Error(err)
	}

	firmwareFile = fileName
	w.Header().Set("Content-Type", "application/json; charset=UTF-8")
	//w.WriteHeader(http.StatusOK)
	ToJSON(GenericResponse(err), w)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) esp32Handler(w http.ResponseWriter, r *http.Request) {
	if r.Header.Get("Content-Type") != "" {
		value, _ := header.ParseValueAndParams(r.Header, "Content-Type")
		if value != "application/json" {
			web.logger.Error("Content-Type header is not application/json")
			return
		}
	}

	if r.Method == "POST" {

		msg := map[string]interface{}{}
		body, err := ioutil.ReadAll(r.Body)
		if err != nil {
			web.logger.Error(err)
			return
		}
		err = json.Unmarshal(body, &msg)
		if err != nil {
			web.logger.Error(err)
			return
		}

		//web.logger.Info(msg)
		Esp32MAC := msg["DEVICE"].(string)

		if s.ToLower(web.config.ControlMAC) == "no" ||
			(s.ToLower(web.config.ControlMAC) == "yes" && Esp32MAC == web.config.MAC) {
			ctx := context.Background()
			switch msg["ORDER"] {
			case "LOGING":
				err = web.repo.AddLoging(ctx, msg["MESSAGE"],
					msg["TYPE"], msg["DATE_TIME"])
				if err != nil {
					web.logger.Errorf("add loging error: %v", err)
				}
			case "DS18B20":
				err = web.repo.AddDs18b20(ctx, msg["TEMPR"])
				if err != nil {
					web.logger.Errorf("add ds18b20 error: %v", err)
				}
			case "BME280":
				err = web.repo.AddBmx280(ctx, msg["PRESS"], msg["TEMPR"], msg["HUM"])
				if err != nil {
					web.logger.Errorf("add bmx280 error: %v", err)
				}
			case "RADSENS":
				err = web.repo.AddRadsens(ctx, msg["RID"], msg["RIS"], msg["PULSE"])
				if err != nil {
					web.logger.Errorf("add radsens error: %v", err)
				}
			case "ZE08CH2O":
				err = web.repo.AddZe08ch2o(ctx, msg["CH2O"])
				if err != nil {
					web.logger.Errorf("add radsens error: %v", err)
				}
			case "GET_SETTINGS":
				if res, err := web.repo.GetEsp32Settings(ctx, msg["DATE_TIME"]); err != nil {
					web.logger.Errorf("get settings error: %v", err)
				} else {
					var data struct {
						ValveState         uint8   `json:"valve_state"`
						CCS811Baseline     uint8   `json:"ccs811_baseline"`
						MinTempn           float64 `json:"min_temp"`
						MaxTemp            float64 `json:"max_temp"`
						ValveDisable       uint8   `json:"valve_disable"`
						SetupMode          uint8   `json:"setup_mode"`
						Reboot             uint8   `json:"reboot"`
						RadsensHVMode      uint8   `json:"radsens_hv_mode"`
						RadsensHVState     uint8   `json:"radsens_hv_state"`
						RadsensSensitivity uint8   `json:"radsens_sensitivity"`
						Firmware           string  `json:"firmware"`
						ClearJournalEsp32  uint8   `json:"clear_journal_esp32"`
					}
					data.ValveState = toUint8(res.ValveState)
					data.CCS811Baseline = uint8(res.CCS811Baseline)
					data.MinTempn = res.MinTempn
					data.MaxTemp = res.MaxTemp
					data.ValveDisable = toUint8(res.ValveDisable)
					data.Firmware = res.Firmware
					data.SetupMode = toUint8(res.SetupMode)
					data.Reboot = toUint8(res.Reboot)
					data.RadsensHVMode = toUint8(res.RadsensHVMode)
					data.RadsensHVState = toUint8(res.RadsensHVState)
					data.RadsensSensitivity = uint8(res.RadsensSensitivity)
					data.ClearJournalEsp32 = toUint8(res.ClearJournalEsp32)
					ToJSON(&data, w)
				}
			case "RADSENS_HV_SET":
				err = web.repo.SetHVRadsens(ctx, msg["STATE"])
				if err != nil {
					web.logger.Errorf("set radsens HV error: %v", err)
				}
			case "RADSENS_SENSITIVITY_SET":
				err = web.repo.SetSensRadsens(ctx, msg["SENSITIVITY"])
				if err != nil {
					web.logger.Errorf("set radsens sensitivity error: %v", err)
				}
			case "AP_MODE_ON":
				err = web.repo.SetAccesPointMode(ctx)
				if err != nil {
					web.logger.Errorf("esp32 set acceess point mode error: %v", err)
				}
			case "REBOOTED":
				err = web.repo.Esp32Reboot(ctx)
				if err != nil {
					web.logger.Errorf("esp32 reboot error: %v", err)
				}
			case "UPGRADE_FAIL":
				err = web.repo.TerminateUpgrade(ctx)
				if err != nil {
					web.logger.Errorf("esp32 upgrade fail: %v", err)
				}
				err = os.Remove(firmwareFile)
				if err != nil {
					web.logger.Error(err)
				}
			case "UPGRADE_SUCCESS":
				err = web.repo.SuccessUpgrade(ctx)
				if err != nil {
					web.logger.Errorf("esp32 upgrade fail: %v", err)
				}
				err = os.Remove(firmwareFile)
				if err != nil {
					web.logger.Error(err)
				}
			case "JOURNAL_CLEARED":
				err = web.repo.JournaCleared(ctx)
				if err != nil {
					web.logger.Errorf("esp32 journal clear fail: %v", err)
				}
			default:
				web.logger.Errorf("Unknown ESP32 order: %s\n", msg["ORDER"])
			}
		} else {
			web.logger.Errorf("Unknown device: %s\n", msg["DEVICE"])
		}
	} else {
		web.logger.Error("Unsuported GET method for ESP32")
	}
	go web.metrics.ResponceCode(http.StatusOK)
}

func toUint8(b bool) (ret uint8) {
	ret = 0
	if b {
		ret = 1
	}
	return
}
