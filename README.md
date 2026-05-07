# react-native-nitro-cardinal-direction

This is a React Native nitro module that allows realtime updates for the magnetic cardinal direction the user is currently facing in.

## Installation

TODO: Add to npm

`npm install react-native-nitro-cardinal-direction`

## Usage

### Importing Module

`import { cardinalDirection } from 'react-native-nitro-cardinal-direction';`

### Starting the listener

```
      cardinalDirection.startUpdates((data) => {
        console.log(data);
      });
```

The data returned by the listner is a object containing timestamp of the last update, heading in degrees and the direction you are facing.

```
{
  timestamp: number,
  heading: number,
  direction: string
}
```

### Stopping the listener

```
      cardinalDirection.stopUpdates();
```
