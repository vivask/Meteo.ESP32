package app

import (
	"context"
	"fmt"
	"meteo-daemon/services/data"
)

type SyncEsp32Tables struct {
	repo data.Repository
}

func NewSyncEsp32Tables(repo data.Repository) *SyncEsp32Tables {
	return &SyncEsp32Tables{
		repo: repo,
	}
}

func (s *SyncEsp32Tables) Run(params map[string]string) (err error) {
	if len(params) != 0 {
		return fmt.Errorf("Invalid number of parameters: %d, required 0", len(params))
	}
	ctx := context.Background()
	s.repo.SyncDs18b20(ctx)
	s.repo.SyncBmx280(ctx)
	s.repo.SyncZe08ch2o(ctx)
	s.repo.SyncRadsens(ctx)
	return nil
}

func (p *SyncEsp32Tables) Terminate() error {
	return nil
}
