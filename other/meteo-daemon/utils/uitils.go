package utils

import (
	"crypto/md5"
	"fmt"
	"io"
	"os"
	"strconv"
	"time"
)

func RightTrunc(str string, n int) string {
	len := len(str)
	return str[0 : len-n]
}

func LeftTrunc(str string, n int) string {
	len := len(str)
	return str[n : len-1]
}

func GetMD5FileSum(fName string) (string, error) {
	f, err := os.OpenFile(fName, os.O_RDONLY, 0644)
	if err != nil {
		return "", err
	}
	defer f.Close()
	fSum := md5.New()
	_, err = io.Copy(fSum, f)
	return fmt.Sprintf("%X", fSum.Sum(nil)), nil
}

func GetMD5StringlnSum(str string) string {
	b := []byte(str + "\n")
	return fmt.Sprintf("%X", md5.Sum(b))
}

func StringToFloat64(str string) (float64, error) {
	return strconv.ParseFloat(str, 64)
}

func StringToFloat32(str string) (float32, error) {
	value, err := strconv.ParseFloat(str, 32)
	return float32(value), err
}

func FloatToString(input_num float64) string {
	return strconv.FormatFloat(input_num, 'f', 1, 64)
}

func FloatToIntString(input_num float64) string {
	num := int(input_num)
	return strconv.Itoa(num)
}

func EmptyToNil(val interface{}) interface{} {
	if val == "" {
		return nil
	} else {
		return val
	}
}

func EmptyToOne(val interface{}) interface{} {
	if val == "" {
		return 1
	}
	dig, _ := strconv.ParseInt(val.(string), 10, 32)
	if dig == 0 {
		return 1
	}
	return val
}

func GetString(key interface{}) (string, error) {
	buf, ok := key.([]byte)
	if !ok {
		return "", fmt.Errorf("Convertation error")
	}
	return string(buf), nil
}

func GetDateTime(d, t string) (time.Time, error) {
	dts := fmt.Sprintf("%s %s", d, t)
	if len(t) == 0 {
		dts = fmt.Sprintf("%s 00:00:00", d)
	}
	return time.ParseInLocation("2006-01-02 15:04:05", dts, time.Local)
}

func StringToBoolean(str string) bool {
	if str == "true" || str == "TRUE" || str == "1" {
		return true
	}
	return false
}

func Expired(d, t string) (bool, error) {
	var sdt string
	now := time.Now()
	if len(t) == 0 && len(d) != 0 {
		sdt = d + " " + "00:00:00"
		goto FINISH
	} else if len(d) == 0 && len(t) != 0 {
		date := now.Format("2006-01-02")
		sdt = date + " " + t
		goto FINISH
	} else if len(t) == 0 && len(d) == 0 {
		return false, fmt.Errorf("Не определены время и дата")
	} else {
		sdt = d + " " + t
		goto FINISH
	}

FINISH:
	dt, err := time.ParseInLocation("2006-01-02 15:04:05", sdt, time.Local)
	if err != nil {
		return false, err
	}
	if dt.Unix()-now.Unix() < 0 {
		return true, nil
	}

	return false, nil
}

func Error(err error) map[string]interface{} {
	return map[string]interface{}{"err": err}
}

func StartListenService(f func() error, duration int, title string) error {
	timeout := time.After(time.Duration(duration) * time.Second)
	fErr := make(chan error, 1)
	go func(fErr chan error) {
		fErr <- f()
	}(fErr)

	select {
	case e := <-fErr:
		if e != nil {
			return fmt.Errorf("%s: failed to start", title)
		}
	case <-timeout:
	}
	return nil
}
