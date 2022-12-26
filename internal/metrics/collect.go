package metrics

import (
	goErr "errors"
	"fmt"
	"strconv"
	"strings"
	"time"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func UpdateCPUUsage(instance *robotv1alpha1.MetricsCollector, cpuUtil *robotv1alpha1.CPUUtilization, output string) error {

	var cpuUsage float64 = 0.0
	if cpuUtil.Value != "" {
		oldCpuUsage, err := strconv.ParseFloat(cpuUtil.Value, 64)
		if err != nil {
			return err
		}
		cpuUsage = oldCpuUsage
	}

	outputFloat64, err := strconv.ParseFloat(output, 64)
	if err != nil {
		return err
	}

	elapsedTimeNano := float64(time.Now().UnixNano() - instance.Status.LastUpdateTimestamp.UnixNano())
	corePercentage := fmt.Sprintf("%.2f", (outputFloat64-cpuUsage)*100/elapsedTimeNano) + "%"
	hostPercentage := fmt.Sprintf("%.2f", (outputFloat64-cpuUsage)*100/float64(instance.Status.Allocatable.Cpu().Value())/elapsedTimeNano) + "%"

	cpuUtil.Value = output
	cpuUtil.CorePercentage = corePercentage
	cpuUtil.HostPercentage = hostPercentage
	cpuUtil.Message = "active"

	return nil
}

func UpdateMemoryUsage(instance *robotv1alpha1.MetricsCollector, memUtil *robotv1alpha1.MemoryUtilization, output string) error {

	outputFloat64, err := strconv.ParseFloat(output, 64)
	if err != nil {
		return err
	}

	hostPercentage := fmt.Sprintf("%.2f", outputFloat64*100/instance.Status.Allocatable.Memory().AsApproximateFloat64()) + "%"

	memUtil.Value = output
	memUtil.HostPercentage = hostPercentage
	memUtil.Message = "active"

	return nil
}

func UpdateNetworkUsage(instance *robotv1alpha1.MetricsCollector, netUtil *robotv1alpha1.NetworkLoadUtilization, output []string) error {

	for _, niStr := range output {

		niData := strings.Split(niStr, ":")
		if len(niData) != 3 {
			return goErr.New("broken network data: " + niStr)
		}

		interfaceName := niData[0]
		interfaceReceive := niData[1]
		interfaceTransmit := niData[2]

		updated := false
		for k, iface := range netUtil.Interfaces {
			if interfaceName == iface.Name {
				err := updateNetworkInterfaceStatus(instance, &iface, interfaceReceive, interfaceTransmit)
				if err != nil {
					return err
				}
				netUtil.Interfaces[k] = iface
				updated = true
				break
			}
		}

		if !updated {
			netUtil.Interfaces = append(netUtil.Interfaces, robotv1alpha1.NetworkInterfaceUtilization{
				Name: interfaceName,
				Receive: robotv1alpha1.NetworkLoad{
					Value: interfaceReceive,
				},
				Transmit: robotv1alpha1.NetworkLoad{
					Value: interfaceTransmit,
				},
			})
		}

	}

	return nil
}

func updateNetworkInterfaceStatus(instance *robotv1alpha1.MetricsCollector, netInterface *robotv1alpha1.NetworkInterfaceUtilization, receive string, transmit string) error {

	elapsedTimeSeconds := float64(time.Now().Sub(instance.Status.LastUpdateTimestamp.Time).Seconds())

	oldReceiveFloat, err := strconv.ParseFloat(netInterface.Receive.Value, 64)
	if err != nil {
		return err
	}

	receiveFloat, err := strconv.ParseFloat(receive, 64)
	if err != nil {
		return err
	}

	oldTransmitFloat, err := strconv.ParseFloat(netInterface.Transmit.Value, 64)
	if err != nil {
		return err
	}

	transmitFloat, err := strconv.ParseFloat(transmit, 64)
	if err != nil {
		return err
	}

	netInterface.Receive.Value = fmt.Sprintf("%.2f", receiveFloat)
	netInterface.Transmit.Value = fmt.Sprintf("%.2f", transmitFloat)

	netInterface.Receive.Load = calculateUnit(receiveFloat, oldReceiveFloat, elapsedTimeSeconds)
	netInterface.Transmit.Load = calculateUnit(transmitFloat, oldTransmitFloat, elapsedTimeSeconds)

	netInterface.Receive.LoadKBit = calculateKBit(receiveFloat, oldReceiveFloat, elapsedTimeSeconds)
	netInterface.Transmit.LoadKBit = calculateKBit(transmitFloat, oldTransmitFloat, elapsedTimeSeconds)

	return nil
}

func calculateUnit(newVal float64, oldVal float64, elapsedTime float64) string {
	loadBytes := (newVal - oldVal) / elapsedTime

	var absoluteVal float64 = loadBytes
	var unit string = "Bytes/s"

	if int(loadBytes)%125 > 1 {

		if int(loadBytes)%125000 > 1 {

			if int(loadBytes)%125000000 > 1 {

				absoluteVal = loadBytes / 125000000
				unit = "gBits/s"

			} else {

				absoluteVal = loadBytes / 125000
				unit = "mBits/s"

			}

		} else {

			absoluteVal = loadBytes / 125
			unit = "kBits/s"

		}

	}

	return fmt.Sprintf("%.2f", absoluteVal) + " " + unit
}

func calculateKBit(newVal float64, oldVal float64, elapsedTime float64) string {
	loadBytes := (newVal - oldVal) / elapsedTime

	absoluteVal := loadBytes / 125
	unit := "kBits/s"

	return fmt.Sprintf("%.2f", absoluteVal) + " " + unit
}
