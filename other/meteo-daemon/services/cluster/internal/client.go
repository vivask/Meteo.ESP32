package pkg

import (
	"fmt"
	"io"
	"net"
	"strings"
	"time"

	"github.com/sirupsen/logrus"
)

type UdpClient struct {
	logger  *logrus.Entry
	address string
}

func NewUdpClient(lg *logrus.Entry, addr string) *UdpClient {
	return &UdpClient{
		logger:  lg,
		address: addr,
	}
}

func (c *UdpClient) Dial(msg string) (resp string, err error) {
	raddr, err := net.ResolveUDPAddr("udp", c.address)
	if err != nil {
		return
	}

	conn, err := net.DialUDP("udp", nil, raddr)
	if err != nil {
		return
	}
	defer conn.Close()
	doneChan := make(chan error, 1)
	reader := strings.NewReader(msg)
	r := make(chan string)
	go func() {
		n, err := io.Copy(conn, reader)
		if err != nil {
			doneChan <- err
			return
		}

		c.logger.Debugf("udp client packet-written: bytes=%d", n)

		buffer := make([]byte, maxBufferSize)

		deadline := time.Now().Add(timeout)
		err = conn.SetReadDeadline(deadline)
		if err != nil {
			doneChan <- err
			return
		}

		nRead, addr, err := conn.ReadFrom(buffer)
		if err != nil {
			doneChan <- err
			return
		}

		c.logger.Debugf("udp client packet-received: bytes=%d from=%s", nRead, addr.String())

		r <- string(buffer[:nRead])
	}()

	timer := time.After(timeout)
	select {
	case <-timer:
		err = fmt.Errorf("udp dial time out")
	case err = <-doneChan:
	case resp = <-r:
	}

	return
}
