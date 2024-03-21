package test

import (
	"time"

	"sigs.k8s.io/controller-runtime/pkg/envtest"
)

func StopTestEnv(testEnv *envtest.Environment) error {
	// Need to sleep if the first stop fails due to a bug:
	// https://github.com/kubernetes-sigs/controller-runtime/issues/1571
	sleepTime := 1 * time.Millisecond
	for i := 0; i < 12; i++ { // Exponentially sleep up to ~4s
		if err := testEnv.Stop(); err == nil {
			return err
		}
		sleepTime *= 2
		time.Sleep(sleepTime)
	}
	return nil
}
