{ stdenv
, mkRosPackage
, robonomics_comm-nightly
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "blockator_agent";
  version = "0.2.0";

  src = ./.;

  propagatedBuildInputs = [
    robonomics_comm-nightly
    python3Packages.pinatapy
    python3Packages.sentry-sdk
    python3Packages.qrcode
  ];

  meta = with stdenv.lib; {
    description = "A Quality Control System for NanoDoctor.pro";
    homepage = http://github.com/vourhey/chemistry-quality-control;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
