# esphome-components
 My custom or improved base components for use in my internal projects. Feel free to use them but they are modified to meet needs of my projects and could change dramatically without letting anyone know.

 ## nextion_simple
 New component which is only ESP => Nextion way for maximum performance and short code

The component keeps Nextion work bounded per ESPHome loop. For busy nodes, the
TX budget can be tightened without disabling two-way features:

```yaml
nextion_simple:
  uart_id: nextion_uart
  # Optional: lower values leave more CPU time for sensors, Wi-Fi and automations.
  tx_max_per_loop: 3
  tx_max_bytes_per_loop: 256
  tx_time_budget_us: 1000
  loop_time_budget_us: 1500
```

`tx_max_per_loop` and `tx_max_bytes_per_loop` cap a burst; `tx_time_budget_us`
limits CPU time spent constructing and queuing it. The UART still transmits
asynchronously. `loop_time_budget_us` is the shared budget for TX and RX work
in one normal component loop pass.

 ## pulse_meter
 Optimized ISR handling for high frequencies

 ## pulse_counter
 Use of new PCNT HW driver and optimized
