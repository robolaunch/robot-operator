package error

import (
	"errors"
	"time"

	ctrl "sigs.k8s.io/controller-runtime"
)

func CheckCreatingOrWaitingError(result *ctrl.Result, err error) error {
	var creatingResourceError *CreatingResourceError
	var waitingForResourceError *WaitingForResourceError
	if !(errors.As(err, &creatingResourceError) || errors.As(err, &waitingForResourceError)) {
		return err
	} else {
		result.Requeue = true
		result.RequeueAfter = 1 * time.Second
		return nil
	}
}
