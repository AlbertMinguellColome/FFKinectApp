# FFLive

(c) Albert Minguell Colome 2016

## Project Layout

Projects will use Cocoapods whenever possible for third party deps.

- **Bangers/**
  - **Classes/**  All created classes, using the `FF` prefix.
    - **Navigation/** Global App Navigation
    - **Screens/** UI, ViewControllers and views.
    - **Engines/** Application engines: Audio, storage, sequencing, marketing, etc.       
    - **API/** REST API Connector           
    - **Models/** Data objects, both Realm based and plain NSObjects.       
    - **Miscellaneous/** Independent code: categories, helpers, etc.
  - **Third Party Code/** Non-cocoapods dependencies, custom components, views, etc.

## Code Format

- Code is formated using clang-format with LLVM standard, 4-space indents, 120-column limit.
- Use clang-formatter with the `file` format defined by `.clang-format`.
    - Use the [Clang-Format](https://github.com/travisjeffery/ClangFormat-Xcode) plugin for XCode.
    - If you want to use command line formatting, Homebrew clang-format for command line formatting with the `-style=file` option.


## Beta builds

- We use Fabric.io for beta distribution. Install the Fabric app if you are going to publish betas.

## Installation steps

- Clone repository into of_v0.X.X_osx_release/apps/myApps folder
- Make sure have previously installed all the addons appearing in addons.make files
- If using Xcode 8 and of_v0.9.3 remove references (within project) of the openFrameworks xcode project files video/ofQt*




# FFKinectApp
# FFKinectApp
