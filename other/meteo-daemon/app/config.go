package app

import (
	"meteo-daemon/services/auth"
	"meteo-daemon/services/cluster"
	"meteo-daemon/services/data"
	"meteo-daemon/services/mail"
	"meteo-daemon/services/mikrotik"
	"meteo-daemon/services/proxy"
	"meteo-daemon/services/scheduler"
	"meteo-daemon/services/telegram"
	"meteo-daemon/services/web"
	"os"
	"reflect"
	"strconv"
	"time"
)

type Config struct {
	Dummy       string
	App         *Configure
	WebServer   *web.Config
	Database    *data.Config
	Mikrotik    *mikrotik.Config
	Telegram    *telegram.Config
	Scheduler   *scheduler.Config
	Mail        *mail.Config
	AuthService *auth.Config
	Proxy       *proxy.Config
	Cluster     *cluster.Config
}

func (c *Config) LoadEnvironment(tag string) {
	checkFields(tag, *c)
}

func checkFields(tag string, iface interface{}) {
	ifv := reflect.ValueOf(iface)
	ift := reflect.TypeOf(iface)

	time.Sleep(100 * time.Millisecond)
	for i := 0; i < ift.NumField(); i++ {
		v := ifv.Field(i)

		switch v.Kind() {
		case reflect.Ptr:
			rvt := v.Elem().Interface()
			tvt1 := reflect.TypeOf(rvt)

			switch tvt1.Kind() {
			case reflect.Ptr, reflect.Struct:
				checkFields(tag, v.Elem().Interface())

			case reflect.String:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						ifv.Field(i).Elem().SetString(value)
					}
				}

			case reflect.Int, reflect.Int64:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseInt(value, 10, 64); err == nil {
							ifv.Field(i).Elem().SetInt(value2)
						}
					}
				}

			case reflect.Int32:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseInt(value, 10, 32); err == nil {
							ifv.Field(i).Elem().SetInt(value2)
						}
					}
				}

			case reflect.Int16:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseInt(value, 10, 16); err == nil {
							ifv.Field(i).Elem().SetInt(value2)
						}
					}
				}

			case reflect.Int8:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseInt(value, 10, 8); err == nil {
							ifv.Field(i).Elem().SetInt(value2)
						}
					}
				}

			case reflect.Uint, reflect.Uint64:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseUint(value, 10, 64); err == nil {
							ifv.Field(i).Elem().SetUint(value2)
						}
					}
				}

			case reflect.Uint32:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseUint(value, 10, 32); err == nil {
							ifv.Field(i).Elem().SetUint(value2)
						}
					}
				}

			case reflect.Uint16:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseUint(value, 10, 16); err == nil {
							ifv.Field(i).Elem().SetUint(value2)
						}
					}
				}

			case reflect.Uint8:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseUint(value, 10, 8); err == nil {
							ifv.Field(i).Elem().SetUint(value2)
						}
					}
				}

			case reflect.Float64:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseFloat(value, 64); err == nil {
							ifv.Field(i).Elem().SetFloat(value2)
						}
					}
				}

			case reflect.Float32:
				envTagName := ift.Field(i).Tag.Get(tag)

				if envTagName != "" {
					value := os.Getenv(envTagName)

					if value != "" {
						if value2, err := strconv.ParseFloat(value, 32); err == nil {
							ifv.Field(i).Elem().SetFloat(value2)
						}
					}
				}

			default:

			}

		case reflect.Struct:
			checkFields(tag, v.Interface())
		}
	}
}
