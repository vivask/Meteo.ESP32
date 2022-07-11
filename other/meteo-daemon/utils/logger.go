package utils

import (
	"os"
	"reflect"

	"github.com/sirupsen/logrus"
)

func InitLogrus(logDir, logFile, logLevel string, file *os.File, entry *logrus.Entry) (*os.File, error) {
	entry.Logger.Formatter = &logrus.TextFormatter{}
	var err error
	var ret *os.File
	if len(logFile) == 0 {
		entry.Logger.SetOutput(file)
	} else {
		//newFile, err := os.OpenFile(logDir+"/"+logFile, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0666)
		newFile, err := os.OpenFile(logDir+"/"+logFile, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0666)
		if err != nil {
			return nil, err
		}
		entry.Logger.SetOutput(newFile)
		ret = newFile
	}
	level, err := logrus.ParseLevel(logLevel)
	if err != nil {
		return nil, err
	}
	entry.Logger.SetLevel(level)
	if level == logrus.DebugLevel {
		entry.Logger.SetReportCaller(true)
	}
	return ret, nil
}

func LogType(tag string, val interface{}, log *logrus.Entry) {
	ift := reflect.TypeOf(val)
	ifv := reflect.ValueOf(val)
	log.Infof("%s: Type: %v, Value: %v", tag, ift, ifv)
}
