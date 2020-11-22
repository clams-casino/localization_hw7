# Building

Build it on the duckiebot. Assuming in the root directory of the repo
```
dts devel build -f -H <duckiebot name>.local
```

# Running fused localization
Use the following command to run fused localization on the duckiebot
```
dts devel run -H <duckiebot name>.local
```
The default launcher is setup to start the `fused_localization_node`, `encoder_localization_node`, and `at_localization_node`.

# Running encoder localization
```
dts devel run -H <duckiebot name>.local --launcher encoder
```
This will only start the `encoder_localization_node`.

# Running april tag localization
```
dts devel run -H <duckiebot name>.local --launcher at
```
This will only start the `at_localization_node`.


# Note to TAs
I was quite anxious and apprehensive about updating my duckiebot, so I tested it with the old encoder driver where the ticks were not timestamped. However, the code works such that if the ticks are timestamps, then the encoder estimate is published with the latest tick time, otherwise it is published with the current ros time.