# Ally Speech Service Enhancements for HowYouSeeMe Integration

## Overview

This document outlines the enhancements to Ally's existing speech service to enable comprehensive audio-visual scene understanding in HowYouSeeMe. Ally already has TTS and STT capabilities - we're extending these with spatial awareness, multi-speaker support, and environmental audio understanding.

## Current Ally Speech Capabilities

✅ **Existing Features:**
- Speech-to-Text (OpenAI Whisper)
- Text-to-Speech (Coqui TTS)
- ggwave Communication
- WebSocket Service with GPU acceleration
- Voice Commands and Hands-free interaction

## Proposed Enhancements

### 1. Sound Source Localization 🎯

**Purpose**: Spatially locate where sounds are coming from in the environment

```python
# New component for Ally speech service
class SoundSourceLocalizer:
    def __init__(self, mic_array_config):
        self.mic_positions = mic_array_config  # Microphone array positions
        self.sample_rate = 16000
        self.frame_size = 1024
        
    def localize_sound(self, audio_channels):
        """Estimate sound source direction using TDOA/beamforming"""
        # Time Difference of Arrival (TDOA) analysis
        tdoa_estimates = self.compute_tdoa(audio_channels)
        
        # Convert to spatial coordinates
        azimuth, elevation = self.tdoa_to_angles(tdoa_estimates)
        
        # Map to HowYouSeeMe coordinate system
        world_coordinates = self.audio_to_world_coords(azimuth, elevation)
        
        return {
            'source_direction': {
                'azimuth': azimuth,
                'elevation': elevation,
                'confidence': self.calculate_confidence(tdoa_estimates)
            },
            'world_position': world_coordinates,
            'timestamp': time.time()
        }
    
    def continuous_localization(self, audio_stream):
        """Continuously track multiple sound sources"""
        for audio_frame in audio_stream:
            locations = self.localize_sound(audio_frame)
            yield locations
```

**Integration with HowYouSeeMe:**
```python
# In world state entity
{
  "entity_id": "human-12",
  "type": "human", 
  "audio_context": {
    "is_speaking": True,
    "speech_direction": [45, 10],  # azimuth, elevation
    "voice_activity_confidence": 0.89,
    "last_speech_timestamp": "2025-09-24T09:20:30Z"
  }
}
```

### 2. Speaker Identification & Diarization 👥

**Purpose**: Identify who is speaking in multi-person environments

```python
class SpeakerDiarization:
    def __init__(self):
        self.voice_embeddings = {}  # Store known speaker embeddings
        self.clustering_model = self.load_speaker_clustering()
        
    def identify_speakers(self, audio_segment):
        """Identify and separate multiple speakers"""
        # Extract speaker embeddings
        embeddings = self.extract_speaker_features(audio_segment)
        
        # Cluster speakers
        speaker_segments = self.cluster_speakers(embeddings)
        
        # Match with known speakers
        identified_speakers = []
        for segment in speaker_segments:
            speaker_id = self.match_known_speaker(segment.embedding)
            identified_speakers.append({
                'speaker_id': speaker_id,
                'start_time': segment.start,
                'end_time': segment.end,
                'confidence': segment.confidence,
                'text': None  # To be filled by STT
            })
        
        return identified_speakers
    
    def register_speaker(self, audio_sample, speaker_name):
        """Register a new speaker's voice profile"""
        embedding = self.extract_speaker_features(audio_sample)
        self.voice_embeddings[speaker_name] = embedding
        return speaker_name
    
    def continuous_diarization(self, audio_stream):
        """Real-time speaker diarization"""
        buffer = AudioBuffer(window_size=3.0)  # 3-second window
        
        for audio_chunk in audio_stream:
            buffer.add(audio_chunk)
            
            if buffer.is_full():
                speakers = self.identify_speakers(buffer.get_audio())
                yield speakers
```

**Integration with HowYouSeeMe:**
```python
# Enhanced human entity with speaker identification
{
  "entity_id": "human-12",
  "type": "human",
  "identity": {
    "face_id": "face-9834",
    "speaker_id": "speaker-john",
    "name": "John",
    "recognition_confidence": 0.87
  },
  "speech_activity": {
    "currently_speaking": True,
    "speech_segments": [
      {
        "start_time": "2025-09-24T09:19:45Z",
        "end_time": "2025-09-24T09:20:12Z", 
        "transcription": "Where did I put the apple?",
        "confidence": 0.92
      }
    ]
  }
}
```

### 3. Audio Event Detection 🔊

**Purpose**: Detect and classify environmental sounds and actions

```python
class AudioEventDetector:
    def __init__(self):
        self.event_classifier = self.load_audio_event_model()
        self.event_classes = {
            'door_slam': 'door_closing',
            'glass_break': 'glass_breaking',
            'footsteps': 'walking',
            'cooking_sounds': 'cooking_activity',
            'appliance_beep': 'appliance_notification',
            'phone_ring': 'phone_ringing',
            'keyboard_typing': 'typing',
            'water_running': 'water_activity'
        }
    
    def detect_events(self, audio_segment):
        """Classify audio events in the segment"""
        # Extract audio features
        features = self.extract_audio_features(audio_segment)
        
        # Classify events
        predictions = self.event_classifier.predict(features)
        
        detected_events = []
        for pred in predictions:
            if pred.confidence > 0.6:  # Confidence threshold
                detected_events.append({
                    'event_type': pred.class_name,
                    'confidence': pred.confidence,
                    'start_time': pred.start_time,
                    'duration': pred.duration,
                    'location_hint': self.estimate_event_location(audio_segment)
                })
        
        return detected_events
    
    def continuous_event_detection(self, audio_stream):
        """Real-time audio event detection"""
        for audio_chunk in audio_stream:
            events = self.detect_events(audio_chunk)
            for event in events:
                yield event
```

**Integration with HowYouSeeMe:**
```python
# Audio events in world state
{
  "audio_events": [
    {
      "event_id": "evt-001",
      "type": "door_slam",
      "confidence": 0.85,
      "timestamp": "2025-09-24T09:20:15Z",
      "estimated_location": [3.2, 1.8, 0.0],
      "context": "Someone left the kitchen area"
    }
  ]
}
```

### 4. Audio-Visual Synchronization 🎬

**Purpose**: Correlate audio and visual events for comprehensive understanding

```python
class AudioVisualSync:
    def __init__(self, world_state_manager):
        self.world_state = world_state_manager
        self.sync_buffer = SynchronizationBuffer()
        
    def correlate_audio_visual(self, audio_events, visual_entities):
        """Find correlations between audio and visual data"""
        correlations = []
        
        for audio_event in audio_events:
            # Find visual entities near audio event location
            nearby_entities = self.find_entities_near_location(
                audio_event['estimated_location'],
                visual_entities,
                radius=1.5  # meters
            )
            
            # Check for logical correlations
            for entity in nearby_entities:
                correlation = self.check_correlation(audio_event, entity)
                if correlation['confidence'] > 0.5:
                    correlations.append(correlation)
        
        return correlations
    
    def check_correlation(self, audio_event, visual_entity):
        """Check if audio event correlates with visual entity"""
        correlation_rules = {
            'footsteps': {'entity_type': 'human', 'activity': 'walking'},
            'cooking_sounds': {'entity_type': 'human', 'activity': 'cooking'},
            'door_slam': {'entity_type': 'object', 'class': 'door'},
            'glass_break': {'entity_type': 'object', 'material': 'glass'}
        }
        
        rule = correlation_rules.get(audio_event['type'])
        if not rule:
            return {'confidence': 0.0}
        
        # Calculate correlation confidence
        confidence = 0.0
        if visual_entity['type'] == rule.get('entity_type'):
            confidence += 0.4
        
        if visual_entity.get('activity') == rule.get('activity'):
            confidence += 0.3
            
        if visual_entity.get('class') == rule.get('class'):
            confidence += 0.3
        
        return {
            'audio_event_id': audio_event['event_id'],
            'visual_entity_id': visual_entity['entity_id'],
            'correlation_type': 'activity_match',
            'confidence': confidence,
            'explanation': f"Audio event {audio_event['type']} matches {visual_entity['type']} activity"
        }
```

## Enhanced Ally Speech Service Architecture

```python
# Enhanced speech service with new capabilities
class EnhancedAllyAudioService:
    def __init__(self):
        # Existing components
        self.whisper_stt = WhisperSTT()
        self.coqui_tts = CoquiTTS()
        self.ggwave = GGWave()
        
        # New components
        self.sound_localizer = SoundSourceLocalizer(mic_array_config)
        self.speaker_diarization = SpeakerDiarization()
        self.event_detector = AudioEventDetector()
        self.audio_visual_sync = AudioVisualSync()
        
        # Integration
        self.howyouseeme_client = HowYouSeeMeClient()
        
    async def process_audio_stream(self, audio_stream):
        """Enhanced audio processing pipeline"""
        async for audio_chunk in audio_stream:
            # Parallel processing of audio
            tasks = [
                self.whisper_stt.transcribe(audio_chunk),
                self.sound_localizer.localize_sound(audio_chunk),
                self.speaker_diarization.identify_speakers(audio_chunk),
                self.event_detector.detect_events(audio_chunk)
            ]
            
            results = await asyncio.gather(*tasks)
            transcription, sound_location, speakers, events = results
            
            # Combine audio analysis
            audio_analysis = {
                'transcription': transcription,
                'sound_location': sound_location,
                'speakers': speakers,
                'audio_events': events,
                'timestamp': time.time()
            }
            
            # Send to HowYouSeeMe for integration
            await self.howyouseeme_client.update_audio_context(audio_analysis)
            
            # Enhanced voice command processing
            if transcription and transcription['confidence'] > 0.8:
                await self.process_enhanced_voice_command(
                    transcription['text'],
                    audio_analysis
                )
    
    async def process_enhanced_voice_command(self, text, audio_context):
        """Process voice commands with spatial and speaker context"""
        # Determine who spoke and from where
        speaker_info = audio_context.get('speakers', [])
        location_info = audio_context.get('sound_location', {})
        
        # Enhanced command processing
        if "where is" in text.lower():
            # Spatial query with speaker context
            await self.handle_spatial_query(text, speaker_info, location_info)
        elif "remember" in text.lower():
            # Memory command with speaker attribution
            await self.handle_memory_command(text, speaker_info, location_info)
```

## MCP Integration Extensions

```python
# New MCP tools for enhanced audio
@mcp_tool("get_audio_scene_analysis")
async def get_audio_scene_analysis():
    """Get comprehensive audio scene understanding"""
    current_audio = ally_audio_service.get_current_audio_analysis()
    
    return {
        'active_speakers': current_audio.get('speakers', []),
        'sound_sources': current_audio.get('sound_locations', []),
        'audio_events': current_audio.get('events', []),
        'speech_activity': {
            'total_speakers': len(current_audio.get('speakers', [])),
            'conversation_active': any(s['currently_speaking'] for s in current_audio.get('speakers', [])),
            'dominant_speaker': current_audio.get('dominant_speaker')
        }
    }

@mcp_tool("correlate_audio_visual")
async def correlate_audio_visual(time_window: int = 30):
    """Find audio-visual correlations in recent time window"""
    correlations = audio_visual_sync.get_recent_correlations(time_window)
    
    return {
        'correlations': correlations,
        'insights': [
            f"Detected {len([c for c in correlations if c['type'] == 'speech_visual'])} speech-visual matches",
            f"Found {len([c for c in correlations if c['type'] == 'activity_audio'])} activity-audio correlations"
        ]
    }

@mcp_tool("identify_speaker_by_location")
async def identify_speaker_by_location(audio_direction: List[float]):
    """Identify speaker based on audio localization and visual data"""
    # Find human entities near the audio source
    visual_humans = world_state_manager.get_entities_by_type('human')
    
    closest_human = None
    min_distance = float('inf')
    
    for human in visual_humans:
        # Calculate angular distance between audio direction and human position
        human_direction = calculate_direction_from_camera(human['pose']['position'])
        angular_distance = calculate_angular_distance(audio_direction, human_direction)
        
        if angular_distance < min_distance:
            min_distance = angular_distance
            closest_human = human
    
    if closest_human and min_distance < 30:  # 30 degrees tolerance
        return {
            'identified_speaker': closest_human['entity_id'],
            'confidence': max(0, 1 - (min_distance / 30)),
            'visual_confirmation': True,
            'audio_direction': audio_direction,
            'visual_direction': calculate_direction_from_camera(closest_human['pose']['position'])
        }
    
    return {
        'identified_speaker': None,
        'confidence': 0,
        'visual_confirmation': False,
        'reason': 'No visual match for audio source'
    }
```

## Installation and Setup

```bash
# Add to Ally speech service dependencies
pip install librosa
pip install pyaudio
pip install speechrecognition
pip install scikit-learn
pip install numpy
pip install scipy

# For advanced audio processing
pip install torchaudio
pip install asteroid  # For sound source separation
pip install webrtcvad  # Voice activity detection
```

## Integration Points

1. **Ally Speech Service Extensions**: Enhance existing WebSocket service with new audio capabilities
2. **HowYouSeeMe Audio Context**: Add audio analysis to world state entities
3. **MCP Audio Tools**: New tools for audio-visual scene understanding
4. **Real-time Synchronization**: Audio events synchronized with visual perception

This enhancement transforms Ally from a speech interface into a comprehensive audio-visual understanding system, perfectly complementing HowYouSeeMe's visual perception capabilities.