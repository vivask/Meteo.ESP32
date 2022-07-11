package main

import (
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"meteo-daemon/app"

	"github.com/BurntSushi/toml"
	"github.com/pborman/getopt"
)

const (
	DEFAULT_WORK_DIR               = "/var/lib/meteo-daemon"
	DEFAULT_CONFIG                 = ".config/config.toml"
	DEFAULT_USE_ENV                = "false"
	ENV_FIELDS_TAG                 = "env"
	SHUTDOWN_TIME    time.Duration = 1 * time.Minute
)

type commandLineParameters struct {
	ConfigPath *string
	EnvEnable  *string
}

// Main

func main() {
	shutdown := make(chan bool)

	params := getCommandLineParameters()

	cnf, err := loadConfig(*params.ConfigPath)
	if err != nil {
		panic(err)
	}
	//fmt.Printf("Config: %v\n", cnf)
	if params.EnvEnable != nil && *params.EnvEnable != DEFAULT_USE_ENV {
		cnf.LoadEnvironment(ENV_FIELDS_TAG)
	}
	cmd, err := app.New(cnf, DEFAULT_WORK_DIR)
	if err != nil {
		panic(err)
	}

	if err = cmd.Start(); err != nil {
		panic(err)
	}

	gracefulStop(shutdown)
	<-shutdown

	go aggressiveStop()

	if err := cmd.Stop(); err != nil {
		panic(err)
	}
}

func getCommandLineParameters() *commandLineParameters {
	params := &commandLineParameters{
		ConfigPath: getopt.StringLong("config", 'c', DEFAULT_WORK_DIR+"/"+DEFAULT_CONFIG, "Path to config file"),
		EnvEnable:  getopt.StringLong("env", 'e', DEFAULT_USE_ENV, "Use environment variables in configuration "),
	}

	getopt.Parse()

	return params
}

func loadConfig(path string) (*app.Config, error) {
	var config app.Config

	_, err := toml.DecodeFile(path, &config)

	return &config, err
}

func gracefulStop(shutdown chan bool) {
	var gracefulStop = make(chan os.Signal)

	signal.Notify(gracefulStop,
		syscall.SIGHUP,
		syscall.SIGINT,
		syscall.SIGTERM,
		syscall.SIGQUIT)

	go func() {
		<-gracefulStop
		// fmt.Printf("caught sig: %+v", sig)
		shutdown <- true
	}()
}

func aggressiveStop() {
	ticker := time.NewTicker(SHUTDOWN_TIME)

	<-ticker.C

	fmt.Println("The web application is aggressive stop")
	os.Exit(0)
}
