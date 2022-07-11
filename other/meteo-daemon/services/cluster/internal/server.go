package pkg

import (
	"context"
	"fmt"
	"net"
	"time"

	"github.com/sirupsen/logrus"
)

const maxBufferSize = 64
const timeout = 1 * time.Second

type UdpServer struct {
	logger  *logrus.Entry
	address string
	ctx     context.Context
}

func NewUdpServer(ctx context.Context, lg *logrus.Entry, addr string, port int) *UdpServer {
	return &UdpServer{
		ctx:     ctx,
		logger:  lg,
		address: fmt.Sprintf("%s:%d", addr, port),
	}
}

func (s *UdpServer) Listen() (err error) {
	pc, err := net.ListenPacket("udp", s.address)
	if err != nil {
		return
	}
	defer pc.Close()
	doneChan := make(chan error, 1)
	buffer := make([]byte, maxBufferSize)
	go func() {
		for {
			n, addr, err := pc.ReadFrom(buffer)
			if err != nil {
				doneChan <- err
				continue
			}

			s.logger.Debugf("udp server packet-received: bytes=%d from=%s", n, addr.String())

			deadline := time.Now().Add(timeout)
			err = pc.SetWriteDeadline(deadline)
			if err != nil {
				doneChan <- err
				continue
			}

			// Write the packet's contents back to the client.
			n, err = pc.WriteTo(buffer[:n], addr)
			if err != nil {
				doneChan <- err
				continue
			}

			s.logger.Debugf("udp server packet-written: bytes=%d to=%s", n, addr.String())
		}
	}()

	select {
	case <-s.ctx.Done():
		err = s.ctx.Err()
	case err = <-doneChan:
	}

	return
}
