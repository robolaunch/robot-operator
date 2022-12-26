package metrics_collector

import (
	"bytes"
	"context"
	"fmt"
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

func (r *MetricsCollectorReconciler) reconcileGetCPUUsage(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("cat /sys/fs/cgroup/cpu/cpuacct.usage")

	for k, c := range instance.Status.ComponentMetrics {
		out, err := r.readValueFromContainer(instance, c.PodReference.Name, c.ContainerName, cmdBuilder.String())
		if err != nil {
			c.CPUUtilization = robotv1alpha1.CPUUtilization{
				Value:   "0",
				Message: err.Error(),
			}
		} else {

			var oldCpuUsage float64 = 0.0
			if c.CPUUtilization.Value != "" {
				oldCpuUsage, err = strconv.ParseFloat(c.CPUUtilization.Value, 64)
				if err != nil {
					return err
				}
			}

			elapsedTimeNano := float64(time.Now().UnixNano() - instance.Status.LastUpdateTimestamp.UnixNano())
			corePercentage := fmt.Sprintf("%f", (out-oldCpuUsage)*100/elapsedTimeNano) + "%"
			hostPercentage := fmt.Sprintf("%f", (out-oldCpuUsage)*100/float64(instance.Status.Allocatable.Cpu().Value())/elapsedTimeNano) + "%"

			c.CPUUtilization = robotv1alpha1.CPUUtilization{
				Value:          fmt.Sprintf("%f", out),
				CorePercentage: corePercentage,
				HostPercentage: hostPercentage,
				Message:        "active",
			}
		}

		instance.Status.ComponentMetrics[k] = c
	}

	return nil
}

func (r *MetricsCollectorReconciler) reconcileGetMemoryUsage(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("cat /sys/fs/cgroup/memory/memory.usage_in_bytes")

	for k, c := range instance.Status.ComponentMetrics {
		out, err := r.readValueFromContainer(instance, c.PodReference.Name, c.ContainerName, cmdBuilder.String())
		if err != nil {
			c.MemoryUtilization = robotv1alpha1.MemoryUtilization{
				Value:   "0",
				Message: err.Error(),
			}
		} else {

			hostPercentage := fmt.Sprintf("%f", out*100/instance.Status.Allocatable.Memory().AsApproximateFloat64()) + "%"

			c.MemoryUtilization = robotv1alpha1.MemoryUtilization{
				Value:          fmt.Sprintf("%f", out),
				HostPercentage: hostPercentage,
				Message:        "active",
			}
		}

		instance.Status.ComponentMetrics[k] = c
	}

	return nil
}

func (r *MetricsCollectorReconciler) readValueFromContainer(instance *robotv1alpha1.MetricsCollector, podName string, containerName string, cmd string) (float64, error) {

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
	outputFloat64, err := strconv.ParseFloat(output, 64)
	if err != nil {
		return 0, err
	}

	return outputFloat64, nil
}
