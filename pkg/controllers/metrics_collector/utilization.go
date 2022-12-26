package metrics_collector

import (
	"bytes"
	"context"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/remotecommand"
)

func (r *MetricsCollectorReconciler) reconcileGetCPUs(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("cat /sys/fs/cgroup/cpu/cpuacct.usage")

	for k, c := range instance.Status.ComponentMetrics {
		out, err := r.readValueFromContainer(instance, c.PodReference.Name, c.ContainerName, cmdBuilder.String())
		if err != nil {
			c.CPUUtilization = robotv1alpha1.CPUUtilization{
				Type:    robotv1alpha1.MetricTypeCPU,
				Value:   0,
				Message: err.Error(),
			}
			continue
		}

		elapsedTimeNano := time.Now().UnixNano() - instance.Status.LastUpdateTimestamp.UnixNano()
		logger.Info(strconv.FormatInt(((out-c.CPUUtilization.Value)/elapsedTimeNano)*100, 10) + "%")

		c.CPUUtilization = robotv1alpha1.CPUUtilization{
			Type:       robotv1alpha1.MetricTypeCPU,
			Value:      out,
			Percentage: strconv.Itoa(int(((out-c.CPUUtilization.Value)/elapsedTimeNano)*100)) + "%",
			Message:    "active",
		}

		instance.Status.ComponentMetrics[k] = c
	}

	return nil
}

func (r *MetricsCollectorReconciler) readValueFromContainer(instance *robotv1alpha1.MetricsCollector, podName string, containerName string, cmd string) (int64, error) {

	inspectCommand := r.RESTClient.
		Post().
		Namespace(instance.Namespace).
		Resource("pods").
		Name(podName).
		SubResource("exec").
		VersionedParams(&corev1.PodExecOptions{
			Container: containerName,
			Command:   internal.Bash(cmd),
			Stdin:     true,
			Stdout:    true,
			Stderr:    true,
		}, runtime.NewParameterCodec(r.Scheme))

	exec, err := remotecommand.NewSPDYExecutor(r.RESTConfig, "POST", inspectCommand.URL())
	if err != nil {
		return 0, err
	}

	var buff bytes.Buffer
	err = exec.Stream(remotecommand.StreamOptions{
		Stdin:  os.Stdin,
		Stdout: &buff,
		Stderr: os.Stderr,
		Tty:    false,
	})

	if err != nil {
		return 0, err
	}

	output := string(buff.Bytes()[:])
	output = strings.ReplaceAll(output, "\n", "")
	outputInt64, err := strconv.ParseInt(output, 10, 64)
	if err != nil {
		return 0, err
	}

	return outputInt64, nil
}
