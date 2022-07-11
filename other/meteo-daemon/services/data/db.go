package data

import (
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"

	"github.com/jinzhu/gorm"
	_ "github.com/jinzhu/gorm/dialects/mysql"
	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp   domain.StartStopInterface
	Logger *logrus.Entry
	Conn   *gorm.DB
	Cli    *client.Client
	Lead   *leader.Leader
}

func NewConnection(cnf *Config) (db *gorm.DB, err error) {

	var connStr string
	switch cnf.Driver {
	case "mysql":
		/*connStr = cnf.User + ":" + cnf.Password + "@tcp(" +
		cnf.Host + ":" + strconv.FormatInt(int64(cnf.Port), 10) +
		")/" + cnf.Schema + "?parseTime=true&loc=Local"*/
		connStr = fmt.Sprintf("%s:%s@tcp(%s:%d)/%s?parseTime=true&loc=Local",
			cnf.User, cnf.Password, cnf.Host, cnf.Port, cnf.Schema)
	default:
		return nil, fmt.Errorf("Unsupportet driver: %s", cnf.Driver)
	}

	db, err = gorm.Open(cnf.Driver, connStr)
	if err != nil {
		return nil, fmt.Errorf("Can't connect to database: %w", err)
	}

	return
}

type Config struct {
	Title      string   `toml:"Title"`
	Driver     string   `toml:"Driver"`
	Host       string   `toml:"Host"`
	Port       int      `toml:"Port"`
	Schema     string   `toml:"Schema"`
	User       string   `toml:"User"`
	Password   string   `toml:"Password"`
	Sync       bool     `toml:"Sync"`
	IgnoreSync []string `toml:"IgnoreSync"`
	LogFile    string   `toml:"LogFile"`
	LogLevel   string   `toml:"LogLevel"`
}
