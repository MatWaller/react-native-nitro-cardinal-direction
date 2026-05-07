import { type HybridObject } from 'react-native-nitro-modules'

export interface SensorData {
  timestamp: number,
  heading: number,
  direction: string,
}

export interface CardinalDirection extends HybridObject<{
  ios: 'c++',
  android: 'c++'
}> {
  startUpdates(callback: (data: SensorData) => void): void
  stopUpdates(): void
}