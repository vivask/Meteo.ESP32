package internal

import (
	"hash"
	"time"

	"hash/fnv"
)

func HashString64(str string) uint64 {
	return fnvHashString64(str)
}

func HashString32(str string) uint32 {
	return fnvHashString32(str)
}

func HashTime64(dt time.Time) uint64 {
	dts := dt.Format("2006-01-02 15:04:05")
	return fnvHashString64(dts)
}

func HashTime32(dt time.Time) uint32 {
	dts := dt.Format("2006-01-02 15:04:05")
	return fnvHashString32(dts)
}

func HashNow64() uint64 {
	dts := time.Now().Format("2006-01-02 15:04:05")
	return fnvHashString64(dts)
}

func HashNow32() uint32 {
	dts := time.Now().Format("2006-01-02 15:04:05")
	return fnvHashString32(dts)
}

func fnvHashString32(str string) uint32 {
	var h hash.Hash32
	h = fnv.New32()
	data := []byte(str)
	_, err := h.Write(data)
	if err != nil {
		return 0
	}
	return h.Sum32()
}

func fnvHashString64(str string) uint64 {
	var h hash.Hash64
	h = fnv.New64()
	data := []byte(str)
	_, err := h.Write(data)
	if err != nil {
		return 0
	}
	return h.Sum64()
}
