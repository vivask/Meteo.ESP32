package domain

// Microservice
// Interfaces
// Copyright © 2021 Eduard Sesigin. All rights reserved. Contacts: <claygod@yandex.ru>

type StartStopInterface interface {
	Start() bool
	Stop() bool
	Add() bool
	Done() bool
	Total() int64
	IsReady() bool
	IsRun() bool
}
