package leader

import "sync"

type Leader struct {
	leader      bool
	master      bool
	aliveRemote bool
	mu          sync.Mutex
}

func New(m bool) *Leader {
	return &Leader{
		master:      m,
		leader:      false,
		aliveRemote: false,
	}
}

func (l *Leader) IsLeader() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	ret := l.leader
	return ret
}

func (l *Leader) SetLeader(lead bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.leader = lead
}

func (l *Leader) IsMaster() bool {
	return l.master
}

func (l *Leader) IsAliveRemote() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	ret := l.aliveRemote
	return ret
}

func (l *Leader) SetAliveRemote(alive bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.aliveRemote = alive
}

func (l *Leader) Self() string {
	if l.master {
		return "MASTER"
	}
	return "SLAVE"
}

func (l *Leader) Other() string {
	if l.master {
		return "SLAVE"
	}
	return "MASTER"
}
