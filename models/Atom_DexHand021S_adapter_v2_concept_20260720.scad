
// Atom <-> DexHand021S adapter v2 concept
// Units: mm
$fn=128;

outer_r = 29;
thickness = 12;

module hex_nut_trap(x,y,flat=7,depth=5){
    translate([x,y,thickness-depth])
        cylinder(h=depth, r=flat/(2*cos(30)), $fn=6);
}

module bolt_hole(x,y,d){
    translate([x,y,-1])
        cylinder(h=thickness+2,r=d/2,$fn=64);
}

// main body
difference(){
    cylinder(h=thickness,r=outer_r);

    // central bore
    cylinder(h=thickness+2,r=9,$fn=128);

    // Atom side: 6 x M4 clearance holes PCD49
    for(i=[0:5]){
        a=60*i;
        bolt_hole(24.5*cos(a),24.5*sin(a),4.5);
    }

    // DexHand side: 5 x M2.5 clearance holes PCD34
    for(i=[0:4]){
        a=72*i+18;
        bolt_hole(17*cos(a),17*sin(a),2.8);
    }

    // weight reduction pockets
    for(i=[0:5]){
        a=60*i;
        translate([18*cos(a),18*sin(a),-0.2])
            cylinder(h=3,r=5,$fn=64);
    }
}

// Atom locating boss
difference(){
    translate([0,0,thickness])
        cylinder(h=3,r=22.55);
    translate([0,0,thickness-0.1])
        cylinder(h=3.2,r=16.75);
}

// radial ribs
for(i=[0:5]){
    rotate([0,0,60*i])
    translate([10, -2, 0])
        cube([16,4,12]);
}
