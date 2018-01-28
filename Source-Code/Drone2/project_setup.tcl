# Remember where we start
set top_dir [pwd]

# Keep device settings constant
set FAMILY "MachXO3LF"
set DEVICE "LCMXO3LF-6900C"
set PACKAGE "CABGA256"
set PERFORMANCE "5"
set PART "LCMXO3LF-6900C-5BG256C"
prj_dev set -family $FAMILY -device $DEVICE -package $PACKAGE -performance $PERFORMANCE -part $PART

# Set default implementation folder
prj_impl active impl1

# Try to find where all the verilog files are stored
if { [catch {set src_dir $top_dir/impl1/source}] } {
	puts stderr "Cound not find $top_dir/impl1/source"
	exit 1
}

# We need to find the names of all the verilog files
if { [catch {set src_files [glob $src_dir/*.v]}] } {
	puts stderr "Could not find *.v files using pattern: $src_dir/*.v"
	exit 1
}

# We don't want to include any test benches in synthesis
set condition "*test*.v"
foreach file $src_files {
	if {[string match $condition $file]} {
		prj_src syn_sim -src $file SimulateOnly
	} else {
		prj_src syn_sim -src $file SynthesisAndSimulate
	}
}

prj_run Synthesis -impl impl1
prj_run Map -impl impl1
prj_run PAR -impl impl1
prj_run Export -impl impl1
